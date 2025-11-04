#ifndef DISPLAY_UART_H
#define DISPLAY_UART_H

#include <stdint.h>
#include "serial_cboard.h"
#include "ui_state.h"

// 串口屏模块头文件

// 初始化显示串口（配置 UART 与必要资源）
void display_init(void);

// 更新显示：展示两个电机状态与当前控制模式
void display_update(const motor_status_t* m1, const motor_status_t* m2, control_mode_t mode);

// 在需要时可调用以强制刷新（同 display_update 功能）
void display_refresh_now(void);

#endif // DISPLAY_UART_H