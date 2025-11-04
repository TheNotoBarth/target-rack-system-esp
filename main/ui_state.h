#ifndef UI_STATE_H
#define UI_STATE_H

#include <stdint.h>

// 模式状态机模块头文件

typedef enum {
	MODE_MANUAL = 0,
	MODE_PRESET1,
	MODE_PRESET2,
	MODE_COUNT // keep last
} control_mode_t;

// 回调类型：当模式切换（最终生效）时会被调用
typedef void (*ui_mode_change_cb_t)(control_mode_t new_mode);

// 初始化状态机（创建任务、设置回调等）
void ui_state_init(void);

// 获取/设置当前模式
control_mode_t ui_state_get_mode(void);
void ui_state_set_mode(control_mode_t mode);

// 外部按键事件（可由 ISR、轮询或测试 CLI 调用）
void ui_state_button_event_up(void);
void ui_state_button_event_down(void);
void ui_state_button_event_ok(void);

// 注册模式变更回调
void ui_state_register_mode_change_cb(ui_mode_change_cb_t cb);

#endif // UI_STATE_H