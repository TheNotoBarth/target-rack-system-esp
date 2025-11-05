#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "config.h"
#include "serial_cboard.h"
#include "simulator.h"
#include <math.h>
#include <stdbool.h>

static const char *TAG = "simulator";

// 模拟器实现：维护两个电机的当前状态与目标值，并以 SIM_UPDATE_HZ 的频率
// 逐步逼近目标值，然后打包帧注入到 serial_cboard_process_raw

typedef struct {
	uint16_t angle; // 0-8191
	int16_t speed;  // RPM
	int16_t current; // raw
	uint8_t temp;
	uint8_t id;
} sim_state_t;

static sim_state_t cur_state[2];

// 模拟器内部的 homing 标志（仅在模拟器层面用于产生电流突变）
static bool sim_homed[2] = { false, false };

// 目标由 simulator_on_command 更新（接收来自 serial_cboard_send 的命令）
typedef struct {
	int16_t target_speed;
	int16_t target_position;
	uint8_t control_mode; // 0 speed, 1 position
	uint8_t motor_id;
} sim_target_t;

static sim_target_t targets[2];

// helper: 找到 motor index (0/1) 对应 id
static int find_index_by_id(uint8_t id)
{
	for (int i = 0; i < 2; ++i) if (cur_state[i].id == id) return i;
	return -1;
}

// 外部回调：当 ESP 在 TEST_MODE 下发送命令时 serial_cboard 会调用此函数
void simulator_on_command(const void *cmds_void, size_t cmd_count)
{
	if (!cmds_void || cmd_count == 0) return;
	const motor_command_t *cmds = (const motor_command_t *)cmds_void;
	for (size_t i = 0; i < cmd_count; ++i) {
		const motor_command_t *c = &cmds[i];
		int idx = find_index_by_id(c->motor_id);
		if (idx < 0) {
			// 如果未初始化过该 id，则使用第一个空位
			for (int j = 0; j < 2; ++j) {
				if (cur_state[j].id == 0) { idx = j; break; }
			}
			if (idx < 0) idx = 0; // 强制使用 0
			cur_state[idx].id = c->motor_id;
		}
		targets[idx].target_speed = c->target_speed;
		targets[idx].target_position = c->target_position;
		targets[idx].control_mode = c->control_mode;
		targets[idx].motor_id = c->motor_id;
		ESP_LOGI(TAG, "simulator: received cmd for id=%u mode=%u tgt_speed=%d tgt_pos=%d (idx=%d)",
				 c->motor_id, c->control_mode, c->target_speed, c->target_position, idx);
	}
}

// 初始化默认值（第一次运行时）
static void sim_init_once(void)
{
	static bool inited = false;
	if (inited) return;
	inited = true;
	// 默认 id 1/2
	cur_state[0].id = 1; cur_state[0].angle = 0; cur_state[0].speed = 0; cur_state[0].current = 0; cur_state[0].temp = 30;
	cur_state[1].id = 2; cur_state[1].angle = 4096; cur_state[1].speed = 0; cur_state[1].current = 0; cur_state[1].temp = 30;
	// 默认目标为当前值
	for (int i = 0; i < 2; ++i) {
		targets[i].motor_id = cur_state[i].id;
		targets[i].target_speed = cur_state[i].speed;
		targets[i].target_position = cur_state[i].angle;
		targets[i].control_mode = 0;
	}

	// 在测试模式下，如果启用了电流复位，则对指定的复位电机施加一个缓慢的反向速度，
	// 以便在到达编码 0 时触发电流突变（仅模拟）。
#if RESET_BY_CURRENT_ENABLED
	for (int i = 0; i < 2; ++i) {
		if (cur_state[i].id == RESET_MOTOR_ID) {
			// 小反向速度（RPM），使编码逐渐减小到 0
			targets[i].target_speed = -5; // 可根据需要调整
			sim_homed[i] = false;
			ESP_LOGI(TAG, "simulator: motor id %u will perform startup homing (sim)", cur_state[i].id);
		}
	}
#endif
}

// 将 int 值按 big-endian 放入 buf
static inline void put_be16(uint8_t *buf, uint16_t v)
{
	buf[0] = (v >> 8) & 0xFF;
	buf[1] = v & 0xFF;
}

// 每个周期更新状态并注入帧
static void sim_task(void *arg)
{
	sim_init_once();
	ESP_LOGI(TAG, "simulator started (TEST_MODE=%d) update_hz=%d", TEST_MODE, SIM_UPDATE_HZ);
	const float hz = (float)SIM_UPDATE_HZ;
	const float dt = 1.0f / hz;

	// 动力学参数（可调）
	const float max_accel_rpm_per_sec = 2000.0f; // 最大加速度
	const float pos_k = 0.5f; // 位置误差到速度的比例 (RPM per encoder unit)
	const int16_t max_rpm = 4000; // 限幅

	while (1) {
		// 更新每个电机
		for (int i = 0; i < 2; ++i) {
			sim_state_t *s = &cur_state[i];
			sim_target_t *t = &targets[i];

			// 根据 control_mode 决定期望速度
			int16_t desired_speed = t->target_speed;
			if (t->control_mode == 1) {
				// 位置控制：计算最短角度差（编码器单位 0..8191）
				int32_t diff = (int32_t)t->target_position - (int32_t)s->angle;
				// wrap to [-4096,4095]
				while (diff > 4096) diff -= 8192;
				while (diff < -4096) diff += 8192;
				// 把位置误差映射为速度
				float v = diff * pos_k;
				if (v > max_rpm) v = max_rpm;
				if (v < -max_rpm) v = -max_rpm;
				desired_speed = (int16_t)v;
			}

			// 限制加速度：每秒 max_accel_rpm_per_sec
			float max_delta_per_tick = max_accel_rpm_per_sec * dt;
			float delta = (float)desired_speed - (float)s->speed;
			if (delta > max_delta_per_tick) delta = max_delta_per_tick;
			if (delta < -max_delta_per_tick) delta = -max_delta_per_tick;
			s->speed = (int16_t)roundf((float)s->speed + delta);

			// 更新角度： RPM -> 编码器单位增量
			// RPM -> rev/s = RPM/60, rev/s * 8192 = units/s
			float units_per_sec = ((float)s->speed) / 60.0f * 8192.0f;
			float units_step = units_per_sec * dt;
			int32_t prev_angle = s->angle;
			int32_t candidate_angle = prev_angle + (int32_t)roundf(units_step);


			// 先计算基础模拟电流：与速度变化相关（简单模型）
			int16_t simulated_current = (int16_t)roundf(delta * 0.5f); // scale

			// 对于相对编码（如 M3508，配置为 RESET_MOTOR_ID），不允许环绕，严格 clamp 到 [0,8191]
			if (s->id == RESET_MOTOR_ID) {
				if (candidate_angle < 0) {
					// 到达下限，保持为 0（不环绕）
					s->angle = 0;
					// 如果仍在向负方向移动，模拟堵转/碰撞导致电流显著增大（恒定 bump，而非累加）
					if (s->speed < 0) {
						int bump = 1000 + (int)(abs(s->speed) / 2);
						if (bump > 2000) bump = 2000;
						// 直接设置为 bump（保留方向由速度决定）
						simulated_current = (s->speed < 0) ? -bump : bump;
					}
				} else if (candidate_angle > 8191) {
					// 到达上限，保持为 8191（不环绕）
					s->angle = 8191;
					if (s->speed > 0) {
						int bump = 1000 + (int)(abs(s->speed) / 2);
						if (bump > 2000) bump = 2000;
						simulated_current = (s->speed < 0) ? -bump : bump;
					}
				} else {
					s->angle = (uint16_t)candidate_angle;
				}
			} else {
				// 绝对编码或其他电机，保持原来的环绕行为
				int32_t new_angle = candidate_angle % 8192;
				if (new_angle < 0) new_angle += 8192;
				s->angle = (uint16_t)new_angle;
			}

			// 支持两种复位模拟方式：电流突增（RESET_BY_CURRENT_ENABLED）或微动开关触发（RESET_BY_SWITCH_ENABLED）
#if RESET_BY_CURRENT_ENABLED
			if (s->id == RESET_MOTOR_ID) {
				// 当未标记为 homed 且到达接近开关位置/处于边界并产生较高电流时触发
				if (!sim_homed[i]) {
					// 如果电流已经被模拟成很高（例如刚才因为边界导致的 bump），触发 homing
					if (abs(simulated_current) >= RESET_CURRENT_RAW_THRESHOLD || s->angle == 0 || s->angle <= RESET_SWITCH_ANGLE_THRESHOLD) {
						simulated_current = 2000;
						sim_homed[i] = true;
						// 停止移动
						s->speed = 0;
						targets[i].target_speed = 0;
						ESP_LOGI(TAG, "simulator: motor id %u simulated homing reached -> current spike", s->id);
					}
				}
			}
#endif

#if RESET_BY_SWITCH_ENABLED
			if (s->id == RESET_MOTOR_ID) {
				if (!sim_homed[i] && s->angle <= RESET_SWITCH_ANGLE_THRESHOLD) {
					// 模拟微动开关触发
					sim_homed[i] = true;
					s->speed = 0;
					targets[i].target_speed = 0;
					ESP_LOGI(TAG, "simulator: motor id %u simulated homing reached -> switch triggered", s->id);
				}
			}
#endif
			// clamp 到 -2000..2000（代表 -20A..20A, 需与上位解析对应）
			if (simulated_current > 2000) simulated_current = 2000;
			if (simulated_current < -2000) simulated_current = -2000;
			s->current = simulated_current;

			// 温度慢慢随电流升高
			int tmp = s->temp + (abs(s->current) / 500); // small change per tick
			if (tmp > 100) tmp = 100;
			if (tmp < 20) tmp = 20;
			s->temp = (uint8_t)tmp;
		}

		// 构建 payload（两个电机，每个 8 字节）
		uint8_t payload[16];
		for (int i = 0; i < 2; ++i) {
			sim_state_t *s = &cur_state[i];
			int base = i * 8;
			put_be16(payload + base + 0, s->angle);
			put_be16(payload + base + 2, (uint16_t)s->speed);
			put_be16(payload + base + 4, (uint16_t)s->current);
			payload[base + 6] = s->temp;
			payload[base + 7] = s->id;
		}

		uint8_t frame[2 + 1 + 16 + 1];
		frame[0] = 0xAA; frame[1] = 0x55; frame[2] = 16;
		memcpy(frame + 3, payload, 16);
		uint8_t ssum = 0;
		for (int i = 0; i < 16; ++i) ssum += payload[i];
		frame[3 + 16] = ssum;

		// 注入解析器
		serial_cboard_process_raw(frame, sizeof(frame));

		// 等待下一周期
		vTaskDelay(pdMS_TO_TICKS((uint32_t)(1000.0f / hz)));
	}
}

void simulator_start(void)
{
	xTaskCreate(sim_task, "sim_task", 4096, NULL, 5, NULL);
}
