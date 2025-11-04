#include <stdio.h>
#include "config.h"
#include "serial_cboard.h"
#include "simulator.h"
#include "ui_state.h"
#include "display_uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "string.h"

static const char *TAG = "app_main";

static void mode_change_cb(control_mode_t new_mode)
{
	const char *names[] = {"MANUAL", "PRESET1", "PRESET2"};
	printf("[MODE_CB] new mode = %s\n", names[new_mode]);
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

    ESP_LOGI(TAG, "app_main finished init");
    // 主任务不退出
    while (1) vTaskDelay(pdMS_TO_TICKS(10000));
}