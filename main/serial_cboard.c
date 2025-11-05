#include "serial_cboard.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "config.h"
#if TEST_MODE
#include "simulator.h"
#endif
#include "ui_state.h"

static const char *TAG = "serial_cboard";

// 双电机状态维护
static motor_status_t motor1;
static motor_status_t motor2;

// 用于保护对 motor1/motor2 的并发访问
#include "freertos/semphr.h"
static SemaphoreHandle_t motor_lock = NULL;

// 返回指定 id 的电机状态（线程安全：复制到静态缓冲区并返回指针）
const motor_status_t* get_motor_status(uint8_t id)
{
	// 注意：之前实现使用了一个单一的静态缓冲区并返回其地址，
	// 导致连续两次调用 get_motor_status(1) 和 get_motor_status(2)
	// 返回的指针指向同一缓冲区，被后一次调用覆盖，进而使
	// 上层代码误以为两台电机状态相同。这里改为直接返回
	// 指向内部静态对象 motor1/motor2 的只读指针。
	// 读取时仍短时加锁以保证一致性（但返回后调用者需
	// 视为只读并接受可能的并发更新）。
	const motor_status_t *ret = NULL;
	if (motor_lock) xSemaphoreTake(motor_lock, portMAX_DELAY);
	if (id == motor1.motor_id) {
		ret = &motor1;
	} else if (id == motor2.motor_id) {
		ret = &motor2;
	} else {
		ret = NULL;
	}
	if (motor_lock) xSemaphoreGive(motor_lock);
	return ret;
}

// UART 配置
#define SERIAL_PORT_NUM      UART_NUM_1
#define SERIAL_TX_GPIO       17
#define SERIAL_RX_GPIO       18
#define SERIAL_BAUD_RATE     115200
#define SERIAL_RX_BUF_SIZE   2048

// 帧定义： [0xAA,0x55][len][payload...][cksum]
// payload 可为 16 字节（两个电机状态）或其它，由 len 指定
static const uint8_t FRAME_HDR0 = 0xAA;
static const uint8_t FRAME_HDR1 = 0x55;

// helpers
static uint8_t calc_cksum(const uint8_t *payload, size_t len)
{
	uint32_t s = 0;
	for (size_t i = 0; i < len; ++i) s += payload[i];
	return (uint8_t)(s & 0xFF);
}

// 解析 payload 为 motor_status（每组 8 字节）
// 用于记录哪些电机已完成复位（防止重复触发）
static bool motor_homed_map[256] = { false };

static void parse_and_print_status(const uint8_t *payload, size_t payload_len)
{
	// 每个电机8字节：angle(2), speed(2), current(2), temp(1), id(1)
	const size_t per = 8;
	size_t count = payload_len / per;
	for (size_t i = 0; i < count; ++i) {
		const uint8_t *p = payload + i * per;
		motor_status_t st;
		st.angle = (uint16_t)p[0] << 8 | p[1];
		st.speed = (int16_t)((uint16_t)p[2] << 8 | p[3]);
		st.current = (int16_t)((uint16_t)p[4] << 8 | p[5]);
		st.temperature = p[6];
		st.motor_id = p[7];

		// 检测是否需要触发电流复位（仅在配置启用时）
#if RESET_BY_CURRENT_ENABLED
		if (st.motor_id == RESET_MOTOR_ID) {
			int16_t abs_curr = (st.current < 0) ? -st.current : st.current;
			if (!motor_homed_map[st.motor_id] && abs_curr >= RESET_CURRENT_RAW_THRESHOLD) {
				// 标记已复位
				motor_homed_map[st.motor_id] = true;
				ESP_LOGI(TAG, "Motor %u reset by overcurrent (raw=%d)", st.motor_id, st.current);
				// 进入手动模式作为正常控制阶段入口
				ui_state_set_mode(MODE_MANUAL);
			}
		}
#endif

		// 更新到对应的全局状态（加锁保护）
		if (motor_lock) xSemaphoreTake(motor_lock, portMAX_DELAY);
		if (st.motor_id == 1) {
			motor1 = st;
		} else if (st.motor_id == 2) {
			motor2 = st;
		}
		if (motor_lock) xSemaphoreGive(motor_lock);

		// 打印 -> 在串口监视器中可见
		// ESP_LOGI(TAG, "Motor %u: angle=%u (0-8191), speed=%d RPM, current=%d (raw), temp=%uC",
		// 		 st.motor_id, st.angle, st.speed, st.current, st.temperature);
	}
}

// 将 raw frame 交给解析器（外部也可调用，用于 TEST_MODE）
void serial_cboard_process_raw(const uint8_t *data, size_t len)
{
	if (len < 4) return;
	if (data[0] != FRAME_HDR0 || data[1] != FRAME_HDR1) return;
	uint8_t paylen = data[2];
	if ((size_t)paylen + 4 != len) {
		ESP_LOGW(TAG, "raw len mismatch: expected %u payload, got %u total", paylen, (uint32_t)len);
		return;
	}
	const uint8_t *payload = data + 3;
	uint8_t cksum = data[3 + paylen];
	if (calc_cksum(payload, paylen) != cksum) {
		ESP_LOGW(TAG, "checksum mismatch");
		return;
	}
	parse_and_print_status(payload, paylen);
}

// 打包并发送到 C 板（将一组 motor_command_t 序列化为 payload）
int serial_cboard_send(const motor_command_t *cmds, size_t cmd_count)
{
	if (!cmds || cmd_count == 0) return -1;
	// 每条命令占 6 字节：target_speed(2), target_pos(2), mode(1), id(1)
	size_t payload_len = cmd_count * 6;
	uint8_t *buf = malloc(payload_len + 4);
	if (!buf) return -1;
	buf[0] = FRAME_HDR0; buf[1] = FRAME_HDR1; buf[2] = (uint8_t)payload_len;
	uint8_t *p = buf + 3;
	for (size_t i = 0; i < cmd_count; ++i) {
		const motor_command_t *c = &cmds[i];
		p[0] = (uint8_t)((uint16_t)c->target_speed >> 8);
		p[1] = (uint8_t)((uint16_t)c->target_speed & 0xFF);
		p[2] = (uint8_t)((uint16_t)c->target_position >> 8);
		p[3] = (uint8_t)((uint16_t)c->target_position & 0xFF);
		p[4] = c->control_mode;
		p[5] = c->motor_id;
		p += 6;
	}
	uint8_t cks = calc_cksum(buf + 3, payload_len);
	buf[3 + payload_len] = cks;

#if TEST_MODE
	// 在测试模式下，打印即将发送的帧内容，不真正发送
	ESP_LOGI(TAG, "TEST_MODE: Frame to send: ");
	for (size_t i = 0; i < payload_len + 4; ++i) {
		printf("%02X ", buf[i]);
	}
	printf("\n");
    // 在测试模式下，通知模拟器更新目标（若模拟器存在）
    if (cmds && cmd_count > 0) {
        // 回调模拟器，将命令数组传过去（注意类型不严格依赖，以避免循环包含复杂性）
        simulator_on_command((const void *)cmds, cmd_count);
    }
	free(buf);
	return 0;
#else
	int w = uart_write_bytes(SERIAL_PORT_NUM, (const char *)buf, (int)(payload_len + 4));
	free(buf);
	if (w <= 0) return -1;
	return 0;
#endif
}

// 发送单个电机命令的便捷函数
int send_motor_command(uint8_t id, int16_t speed, int16_t pos, uint8_t mode)
{
	motor_command_t cmd = {
		.target_speed = speed,
		.target_position = pos,
		.control_mode = mode,
		.motor_id = id
	};
	return serial_cboard_send(&cmd, 1);
}

// UART 接收并解析任务
static void serial_task(void *arg)
{
	uint8_t *data = malloc(SERIAL_RX_BUF_SIZE);
	if (!data) vTaskDelete(NULL);

	while (1) {
#if !TEST_MODE
		int len = uart_read_bytes(SERIAL_PORT_NUM, data, SERIAL_RX_BUF_SIZE, pdMS_TO_TICKS(200));
		if (len <= 0) continue;

		// 简单的流式解析：查找 header
		int idx = 0;
		while (idx + 4 <= len) {
			if (data[idx] == FRAME_HDR0 && data[idx+1] == FRAME_HDR1) {
				uint8_t paylen = data[idx+2];
				size_t framelen = (size_t)paylen + 4;
				if (idx + framelen <= (size_t)len) {
					// 有完整帧
					serial_cboard_process_raw(data + idx, framelen);
					idx += (int)framelen;
					continue;
				} else {
					// 不完整，保留到下次
					break;
				}
			}
			idx++;
		}
#else
		// 在测试模式下，不从物理 UART 读取，而由 simulator 注入 raw
		vTaskDelay(pdMS_TO_TICKS(1000));
#endif
	}

	free(data);
	vTaskDelete(NULL);
}

void serial_cboard_init(void)
{
	// 配置 UART
	const uart_config_t uart_config = {
		.baud_rate = SERIAL_BAUD_RATE,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_APB,
	};
	uart_driver_install(SERIAL_PORT_NUM, SERIAL_RX_BUF_SIZE * 2, 0, 0, NULL, 0);
	uart_param_config(SERIAL_PORT_NUM, &uart_config);
	uart_set_pin(SERIAL_PORT_NUM, SERIAL_TX_GPIO, SERIAL_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

	// 启动解析任务
	xTaskCreate(serial_task, "serial_task", 4096, NULL, 10, NULL);

	// 初始化状态锁
	if (!motor_lock) motor_lock = xSemaphoreCreateMutex();

	ESP_LOGI(TAG, "serial_cboard initialized (UART%d TX=%d RX=%d)", SERIAL_PORT_NUM, SERIAL_TX_GPIO, SERIAL_RX_GPIO);
}
