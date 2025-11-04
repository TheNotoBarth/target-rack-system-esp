#include <stdio.h>
#include "config.h"
#include "serial_cboard.h"
#include "simulator.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "app_main";

void app_main(void)
{
    printf("System Booting... [TEST_MODE=%d]\n", TEST_MODE);
    esp_log_level_set("serial_cboard", ESP_LOG_INFO);
    esp_log_level_set("simulator", ESP_LOG_INFO);

    // 初始化串口通信模块
    serial_cboard_init();

    // 在测试模式下启动模拟器任务以生成并注入模拟帧
    #if TEST_MODE
    simulator_start();
    #endif

    ESP_LOGI(TAG, "app_main finished init");
    // 主任务不退出
    while (1) vTaskDelay(pdMS_TO_TICKS(10000));
}