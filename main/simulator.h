#ifndef SIMULATOR_H
#define SIMULATOR_H

// 测试环境模拟模块头文件

// 在 TEST_MODE 下启动模拟器（周期性注入模拟帧到解析器）
void simulator_start(void);

// 当 ESP 在 TEST_MODE 下发送命令时，serial_cboard 会回调此函数
// 以便模拟器更新目标值。
// cmds: 命令数组（与 serial_cboard.h 中的 motor_command_t 同名）
// cmd_count: 命令数量
void simulator_on_command(const void *cmds, size_t cmd_count);

#endif // SIMULATOR_H