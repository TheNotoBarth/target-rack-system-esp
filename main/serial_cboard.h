#ifndef SERIAL_CBOARD_H
#define SERIAL_CBOARD_H

#include <stdint.h>
#include <stddef.h>

typedef struct {
	uint16_t angle;      // 0-8191 对应 0°-360°
	int16_t speed;       // RPM
	int16_t current;     // 实际电流，单位与C板约定（raw）
	uint8_t temperature; // 摄氏度
	uint8_t motor_id;
} motor_status_t;

typedef struct {
	int16_t target_speed;    // RPM 或相对单位
	int16_t target_position; // 编码器位置或目标位置
	uint8_t control_mode;    // 0=手动速度, 1=位置, ...
	uint8_t motor_id;
} motor_command_t;

// 初始化串口通信（创建任务并启动 UART 驱动）
void serial_cboard_init(void);

// 发送命令到C板（会将命令封装为二进制帧并写入 UART）
int serial_cboard_send(const motor_command_t *cmds, size_t cmd_count);

// 在非硬件环境（TEST_MODE）下，将原始帧数据直接交由解析器处理（用于模拟）
void serial_cboard_process_raw(const uint8_t *data, size_t len);

// 获取指定 id 的电机状态（返回内部静态副本，调用者不可修改）
const motor_status_t* get_motor_status(uint8_t id);

// 发送单个电机命令的便捷函数
int send_motor_command(uint8_t id, int16_t speed, int16_t pos, uint8_t mode);

// 可选：从外部获取最近解析到的电机状态（线程安全性：目前仅供调试）
// 这里我们只提供打印回调，具体状态可在后续扩展

#endif // SERIAL_CBOARD_H