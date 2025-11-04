#include "display_uart.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "config.h"

static const char *TAG = "display_uart";

// 配置（可在 config.h 中覆盖）
#ifndef DISP_UART_NUM
#define DISP_UART_NUM UART_NUM_2
#endif
#ifndef DISP_TX_GPIO
#define DISP_TX_GPIO 9
#endif
#ifndef DISP_RX_GPIO
#define DISP_RX_GPIO 10
#endif
#ifndef DISP_BAUDRATE
#define DISP_BAUDRATE 115200
#endif

#define DISP_TX_BUF_SIZE 1024
#define DISP_RX_BUF_SIZE 256

// 发送低级字符串到屏幕（TEST_MODE 下仅打印）
static esp_err_t display_send_raw(const char *s)
{
	if (!s) return ESP_ERR_INVALID_ARG;
#if TEST_MODE
	// 在测试模式下，直接打印要发送的指令（便于调试）
	ESP_LOGI(TAG, "TEST_MODE: Display will send: %s", s);
	return ESP_OK;
#else
	int len = strlen(s);
	int w = uart_write_bytes(DISP_UART_NUM, s, len);
	if (w != len) {
		ESP_LOGW(TAG, "uart write partial (%d/%d)", w, len);
		return ESP_FAIL;
	}
	return ESP_OK;
#endif
}

// 尝试从屏幕读取 OK\r\n（等待 up to timeout_ms），用于同步（可选）
static bool display_wait_ok(int timeout_ms)
{
#if TEST_MODE
	// 测试模式跳过等待
	(void)timeout_ms;
	return true;
#else
	const TickType_t end = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
	uint8_t buf[16];
	int idx = 0;
	while (xTaskGetTickCount() < end) {
		int r = uart_read_bytes(DISP_UART_NUM, buf + idx, 1, pdMS_TO_TICKS(50));
		if (r > 0) {
			idx += r;
			if (idx >= 4) {
				// 查找 OK\r\n
				for (int i = 0; i <= idx - 4; ++i) {
					if (buf[i] == 'O' && buf[i+1] == 'K' && buf[i+2] == '\r' && buf[i+3] == '\n') {
						return true;
					}
				}
				// 若缓冲已满，重置
				if (idx >= (int)sizeof(buf)) idx = 0;
			}
		}
	}
	return false;
#endif
}

void display_init(void)
{
#if !TEST_MODE
	const uart_config_t uart_config = {
		.baud_rate = DISP_BAUDRATE,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_APB,
	};
	uart_driver_install(DISP_UART_NUM, DISP_RX_BUF_SIZE * 2, 0, 0, NULL, 0);
	uart_param_config(DISP_UART_NUM, &uart_config);
	uart_set_pin(DISP_UART_NUM, DISP_TX_GPIO, DISP_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
#endif
	ESP_LOGI(TAG, "display_init (UART%d TX=%d RX=%d) TEST_MODE=%d", DISP_UART_NUM, DISP_TX_GPIO, DISP_RX_GPIO, TEST_MODE);
}

// 将电机状态与模式信息格式化为 JC 指令并发送
void display_update(const motor_status_t* m1, const motor_status_t* m2, control_mode_t mode)
{
	char buf[512];
	const char *mode_names[] = {"MANUAL", "PRESET1", "PRESET2"};
	const char *mode_str = "?";
	if ((int)mode >= 0 && (int)mode < 3) mode_str = mode_names[mode];

	// 构建显示内容：清屏后显示两路电机信息与模式
	// 示例布局（以 16 号字为主）：
	// 行1: 模式
	// 行2: Motor1 angle/speed/current/temp
	// 行3: Motor2 ...

	int n = 0;
	n += snprintf(buf + n, sizeof(buf) - n, "DIR(0);CLR(0);");
	// 显示模式
	n += snprintf(buf + n, sizeof(buf) - n, "DC16(5,5,'Mode:%s',15);", mode_str);

	if (m1) {
		// angle 0..8191 -> display raw
		n += snprintf(buf + n, sizeof(buf) - n, "DC16(5,25,'M1 A:%u',15);", (unsigned int)m1->angle);
		n += snprintf(buf + n, sizeof(buf) - n, "DC16(90,25,'S:%d',15);", (int)m1->speed);
		n += snprintf(buf + n, sizeof(buf) - n, "DC16(5,45,'I:%d',15);", (int)m1->current);
		n += snprintf(buf + n, sizeof(buf) - n, "DC16(90,45,'T:%uC',15);", (unsigned int)m1->temperature);
	}
	if (m2) {
		n += snprintf(buf + n, sizeof(buf) - n, "DC16(5,65,'M2 A:%u',15);", (unsigned int)m2->angle);
		n += snprintf(buf + n, sizeof(buf) - n, "DC16(90,65,'S:%d',15);", (int)m2->speed);
		n += snprintf(buf + n, sizeof(buf) - n, "DC16(5,85,'I:%d',15);", (int)m2->current);
		n += snprintf(buf + n, sizeof(buf) - n, "DC16(90,85,'T:%uC',15);", (unsigned int)m2->temperature);
	}

	// 最后发送背光设置为中等亮度（示例）并结束行结束符
	n += snprintf(buf + n, sizeof(buf) - n, "BL(100);\r\n");

	// 发送
	display_send_raw(buf);

	// 等待模块准备好（非必须）
	display_wait_ok(200);
}

void display_refresh_now(void)
{
	// 便捷函数：读取当前状态并刷新
	const motor_status_t *m1 = get_motor_status(1);
	const motor_status_t *m2 = get_motor_status(2);
	// ui_state_get_mode 在 ui_state 模块提供
	extern control_mode_t ui_state_get_mode(void);
	control_mode_t m = ui_state_get_mode();
	display_update(m1, m2, m);
}