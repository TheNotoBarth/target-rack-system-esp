#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "config.h"
#include "serial_cboard.h"

static const char *TAG = "simulator";

// 生成一个模拟的 C 板数据帧并交由 serial_cboard 解析
static void generate_and_feed_frame(void)
{
	// payload: 两电机，每个 8 字节 -> 16 字节
	uint8_t payload[16];

	// 模拟电机1
	uint16_t angle1 = 1000; // 示例
	int16_t speed1 = 150;   // RPM
	int16_t cur1 = 200;     // raw
	uint8_t temp1 = 35;
	uint8_t id1 = 1;

	payload[0] = (angle1 >> 8) & 0xFF;
	payload[1] = angle1 & 0xFF;
	payload[2] = (uint16_t)(speed1) >> 8;
	payload[3] = (uint16_t)(speed1) & 0xFF;
	payload[4] = (uint16_t)(cur1) >> 8;
	payload[5] = (uint16_t)(cur1) & 0xFF;
	payload[6] = temp1;
	payload[7] = id1;

	// 模拟电机2
	uint16_t angle2 = 4000;
	int16_t speed2 = -50;
	int16_t cur2 = -120;
	uint8_t temp2 = 34;
	uint8_t id2 = 2;

	payload[8] = (angle2 >> 8) & 0xFF;
	payload[9] = angle2 & 0xFF;
	payload[10] = (uint16_t)(speed2) >> 8;
	payload[11] = (uint16_t)(speed2) & 0xFF;
	payload[12] = (uint16_t)(cur2) >> 8;
	payload[13] = (uint16_t)(cur2) & 0xFF;
	payload[14] = temp2;
	payload[15] = id2;

	// 构建完整帧: hdr(2) len(1) payload(16) cksum(1)
	uint8_t frame[2 + 1 + 16 + 1];
	frame[0] = 0xAA; frame[1] = 0x55; frame[2] = 16;
	memcpy(frame + 3, payload, 16);
	uint8_t s = 0;
	for (int i = 0; i < 16; ++i) s += payload[i];
	frame[3 + 16] = (uint8_t)(s & 0xFF);

	// 直接把 raw 帧交给解析器（不依赖实际 UART）
	serial_cboard_process_raw(frame, sizeof(frame));
}

static void sim_task(void *arg)
{
	ESP_LOGI(TAG, "simulator started (TEST_MODE=%d)", TEST_MODE);
	while (1) {
		generate_and_feed_frame();
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

void simulator_start(void)
{
	xTaskCreate(sim_task, "sim_task", 4096, NULL, 5, NULL);
}
