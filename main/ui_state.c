#include "ui_state.h"
#include "config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include <string.h>
#include <stdint.h>

static const char *TAG = "ui_state";

static control_mode_t s_current_mode = MODE_MANUAL;
static control_mode_t s_last_non_manual = MODE_PRESET1;
static ui_mode_change_cb_t s_mode_cb = NULL;

// 内部：调用回调并打印
static void notify_mode_change(control_mode_t m)
{
	s_current_mode = m;
	const char *names[] = {"MANUAL", "PRESET1", "PRESET2"};
	ESP_LOGI(TAG, "Mode changed -> %s", names[m]);
	if (s_mode_cb) s_mode_cb(m);
}

// 按键逻辑：Up -> 下一个模式；Down -> 上一个模式；OK -> 在 MANUAL 与上次非手动之间切换
void ui_state_button_event_up(void)
{
	control_mode_t next = (s_current_mode + 1) % MODE_COUNT;
	if (next != MODE_MANUAL) s_last_non_manual = next;
	notify_mode_change(next);
}

void ui_state_button_event_down(void)
{
	control_mode_t prev = (s_current_mode + MODE_COUNT - 1) % MODE_COUNT;
	if (prev != MODE_MANUAL) s_last_non_manual = prev;
	notify_mode_change(prev);
}

void ui_state_button_event_ok(void)
{
	if (s_current_mode == MODE_MANUAL) {
		// 切回上次非手动，若无则为 PRESET1
		control_mode_t target = (s_last_non_manual != MODE_MANUAL) ? s_last_non_manual : MODE_PRESET1;
		notify_mode_change(target);
	} else {
		// 切回手动
		notify_mode_change(MODE_MANUAL);
	}
}

control_mode_t ui_state_get_mode(void)
{
	return s_current_mode;
}

void ui_state_set_mode(control_mode_t mode)
{
	if (mode >= MODE_COUNT) return;
	if (mode != MODE_MANUAL) s_last_non_manual = mode;
	notify_mode_change(mode);
}

void ui_state_register_mode_change_cb(ui_mode_change_cb_t cb)
{
	s_mode_cb = cb;
}

// 按键轮询任务实现（在非测试模式下读取GPIO，测试模式由CLI模拟）
static void ui_state_task(void *arg)
{
	// 在此处做简单轮询去抖
	typedef struct {int gpio; int active_level; int last_level; int stable_level; int64_t last_change_ms;} btn_t;
	btn_t buttons[3] = {
		{ BUTTON_UP_GPIO, BUTTON_ACTIVE_LEVEL, 1, 1, 0},
		{ BUTTON_DOWN_GPIO, BUTTON_ACTIVE_LEVEL, 1, 1, 0},
		{ BUTTON_OK_GPIO, BUTTON_ACTIVE_LEVEL, 1, 1, 0}
	};

	while (1) {
		for (int i = 0; i < 3; ++i) {
			int level = gpio_get_level(buttons[i].gpio);
			int64_t now = esp_timer_get_time() / 1000; // ms
			if (level != buttons[i].last_level) {
				buttons[i].last_change_ms = now;
				buttons[i].last_level = level;
			} else {
				if (now - buttons[i].last_change_ms >= BUTTON_DEBOUNCE_MS) {
					if (buttons[i].stable_level != level) {
						buttons[i].stable_level = level;
						// active edge
						if (level == buttons[i].active_level) {
							if (i == 0) ui_state_button_event_up();
							else if (i == 1) ui_state_button_event_down();
							else if (i == 2) ui_state_button_event_ok();
						}
					}
				}
			}
		}
		vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_INTERVAL_MS));
	}
}

void ui_state_init(void)
{
	gpio_config_t io_conf = {};
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pin_bit_mask = ((1ULL<<BUTTON_UP_GPIO) | (1ULL<<BUTTON_DOWN_GPIO) | (1ULL<<BUTTON_OK_GPIO));
	io_conf.pull_up_en = 1;
	io_conf.pull_down_en = 0;
	gpio_config(&io_conf);
	xTaskCreate(ui_state_task, "ui_state_task", 2048, NULL, 7, NULL);
}
