#include <stdio.h>
#include "config.h"
#include "serial_cboard.h"
#include "simulator.h"
#include "ui_state.h"
#include "display_uart.h"
#include "webserver.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "string.h"
#include <math.h>

static const char *TAG = "app_main";

// 预设任务句柄（单例）
static TaskHandle_t s_preset_task = NULL;

static void stop_preset_task(void)
{
	if (s_preset_task) {
		vTaskDelete(s_preset_task);
		s_preset_task = NULL;
	}
}

// PRESET1: GM6020 角度在 0,90,180,270 之间每 2s 跳变一次；
// M3508 在 0 <-> 8191 循环，尝试设置速度为 5
static void preset1_task(void *arg)
{
	(void)arg;
	const int degs[4] = {0, 90, 180, 270};
	int idx = 0;
	while (ui_state_get_mode() == MODE_PRESET1) {
		// GM6020 目标角度 -> 转换为 0-8191 范围
		int deg = degs[idx % 4];
		int pos1 = (int)roundf((deg / 360.0f) * 8191.0f);
		send_motor_command(1, 0, (int16_t)pos1, 1); // mode 1 = position

		// M3508: 设定速度为 5（速度模式），并设置交替位置
		send_motor_command(2, 5, 0, 0); // 尝试设置速度为 5
		// 交替位置 0 / 8191
		int pos2 = (idx % 2 == 0) ? 0 : 8191;
		send_motor_command(2, 0, (int16_t)pos2, 1);

		idx++;
		vTaskDelay(pdMS_TO_TICKS(2000));
	}
	stop_preset_task();
	vTaskDelete(NULL);
}

// PRESET2: GM6020 速度设为 10；M3508 做正弦波运动（周期 4s），每 100ms 更新一次
static void preset2_task(void *arg)
{
	(void)arg;
	const float period_ms = 4000.0f; // 4 秒周期
	const TickType_t delay_ticks = pdMS_TO_TICKS(100);
	// 设置 GM6020 速度为 10
	send_motor_command(1, 10, 0, 0);
	int64_t start = esp_timer_get_time(); // us
	while (ui_state_get_mode() == MODE_PRESET2) {
		int64_t now = esp_timer_get_time();
		float t = (now - start) / 1000.0f; // ms
		float phase = fmodf(t, period_ms) / period_ms; // 0..1
		float s = sinf(2.0f * M_PI * phase);
		float norm = (s * 0.5f) + 0.5f; // 0..1
		int pos = (int)roundf(norm * 8191.0f);
		send_motor_command(2, 0, (int16_t)pos, 1);
		vTaskDelay(delay_ticks);
	}
	stop_preset_task();
	vTaskDelete(NULL);
}

static void mode_change_cb(control_mode_t new_mode)
{
	const char *names[] = {"MANUAL", "PRESET1", "PRESET2"};
	printf("[MODE_CB] new mode = %s\n", names[new_mode]);

	// 切换任务
	if (s_preset_task) {
		// 先停止已有预设任务
		stop_preset_task();
		// 让短暂时间让任务清理
		vTaskDelay(pdMS_TO_TICKS(10));
	}

	if (new_mode == MODE_PRESET1) {
		xTaskCreate(preset1_task, "preset1", 4096, NULL, 5, &s_preset_task);
	} else if (new_mode == MODE_PRESET2) {
		xTaskCreate(preset2_task, "preset2", 4096, NULL, 5, &s_preset_task);
	} else {
		// 切回手动：不做任何自动命令（用户可通过 UI 控制）
	}
}

// CLI 任务：读取 UART0 输入并解析命令
static void cli_task(void *arg)
{
	char buf[128];
	int idx = 0;
	while (1) {
		char c;
		int len = uart_read_bytes(UART_NUM_0, (uint8_t*)&c, 1, pdMS_TO_TICKS(100));
		if (len > 0) {
			if (c == '\n' || c == '\r') {
				buf[idx] = '\0';
				if (idx > 0) {
					// 解析命令
					if (strncmp(buf, "set motor", 9) == 0) {
						char motor[10];
						char param[10];
						int value;
						if (sscanf(buf, "set %s %s %d", motor, param, &value) == 3) {
							uint8_t id = (strcmp(motor, "motor1") == 0) ? 1 : (strcmp(motor, "motor2") == 0) ? 2 : 0;
							if (id == 0) {
								printf("Invalid motor: %s\n", motor);
							} else {
								if (strcmp(param, "speed") == 0) {
									send_motor_command(id, (int16_t)value, 0, 0); // mode 0 for speed
									printf("Set motor%u speed to %d\n", id, value);
								} else if (strcmp(param, "pos") == 0) {
									send_motor_command(id, 0, (int16_t)value, 1); // mode 1 for position
									printf("Set motor%u pos to %d\n", id, value);
								} else {
									printf("Invalid param: %s\n", param);
								}
							}
						} else {
							printf("Invalid command format\n");
						}
					} else {
						// 支持测试模式下的按键模拟："press up" / "press down" / "press ok"
						if (strncmp(buf, "press ", 6) == 0) {
							char which[16];
							if (sscanf(buf, "press %15s", which) == 1) {
								if (strcmp(which, "up") == 0) {
									ui_state_button_event_up();
									printf("Simulated button: UP\n");
								} else if (strcmp(which, "down") == 0) {
									ui_state_button_event_down();
									printf("Simulated button: DOWN\n");
								} else if (strcmp(which, "ok") == 0) {
									ui_state_button_event_ok();
									printf("Simulated button: OK\n");
								} else {
									printf("Unknown press target: %s\n", which);
								}
							} else {
								printf("Invalid press command\n");
							}
						} else {
							printf("Unknown command: %s\n", buf);
						}
					}
				}
				idx = 0;
			} else if (idx < sizeof(buf) - 1) {
				buf[idx++] = c;
			}
		}
	}
	vTaskDelete(NULL);
}

	// 周期性刷新显示任务（移至文件作用域，避免在函数内定义）
	static void display_task(void *arg)
	{
		(void)arg;
		while (1) {
			const motor_status_t *m1 = get_motor_status(1);
			const motor_status_t *m2 = get_motor_status(2);
			control_mode_t cm = ui_state_get_mode();
			display_update(m1, m2, cm);
			vTaskDelay(pdMS_TO_TICKS(1000));
		}
		vTaskDelete(NULL);
	}

void app_main(void)
{
    printf("System Booting... [TEST_MODE=%d]\n", TEST_MODE);
    esp_log_level_set("serial_cboard", ESP_LOG_INFO);
    esp_log_level_set("simulator", ESP_LOG_INFO);

    // 初始化 UART0 用于 CLI
    const uart_config_t uart0_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart0_config);

    // 初始化串口通信模块
    serial_cboard_init();

    // 在测试模式下启动模拟器任务以生成并注入模拟帧
    #if TEST_MODE
    simulator_start();
    #endif

	// 初始化 UI 状态机（按键逻辑）
	ui_state_init();
	ui_state_register_mode_change_cb(mode_change_cb);

    // 启动 CLI 任务
    xTaskCreate(cli_task, "cli_task", 4096, NULL, 5, NULL);

	// 初始化并启动显示模块（会在 TEST_MODE 下仅打印显示命令）
	display_init();

	// 周期性刷新显示（1Hz）
	xTaskCreate(display_task, "display_task", 4096, NULL, 5, NULL);

	// 初始化 Web Server（Wi-Fi AP + HTTP Server）
	webserver_init();

    ESP_LOGI(TAG, "app_main finished init. Access web interface at http://192.168.4.1");
    // 主任务不退出
    while (1) vTaskDelay(pdMS_TO_TICKS(10000));
}