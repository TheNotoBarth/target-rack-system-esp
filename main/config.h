#ifndef CONFIG_H
#define CONFIG_H

#define TEST_MODE 1   // 1表示无硬件测试环境

// 按键与去抖配置（硬件接入时可以在此修改引脚）
#ifndef BUTTON_UP_GPIO
#define BUTTON_UP_GPIO 0
#endif
#ifndef BUTTON_DOWN_GPIO
#define BUTTON_DOWN_GPIO 1
#endif
#ifndef BUTTON_OK_GPIO
#define BUTTON_OK_GPIO 2
#endif

// 按键为低电平按下（默认）
#ifndef BUTTON_ACTIVE_LEVEL
#define BUTTON_ACTIVE_LEVEL 0
#endif

#define BUTTON_DEBOUNCE_MS 50
#define BUTTON_POLL_INTERVAL_MS 50

#endif // CONFIG_H

// Simulator 配置（用于 TEST_MODE）
#ifndef SIM_UPDATE_HZ
#define SIM_UPDATE_HZ 50 // 模拟器更新频率，单位 Hz。可以在需要时修改。
#endif

// 复位相关配置
// 若启用通过电流复位，则当指定电机的 raw current 超过阈值时视为复位触发
#ifndef RESET_BY_CURRENT_ENABLED
#define RESET_BY_CURRENT_ENABLED 1
#endif

#ifndef RESET_MOTOR_ID
// 默认将 ID=2 的电机视为需复位的 M3508（可按需修改）
#define RESET_MOTOR_ID 2
#endif

#ifndef RESET_CURRENT_RAW_THRESHOLD
// raw 值范围与 simulator 保持一致：-2000..2000 对应 -20A..20A
#define RESET_CURRENT_RAW_THRESHOLD 1500
#endif

// 是否启用微动开关复位的模拟（当编码到达指定角度时视为触发）
#ifndef RESET_BY_SWITCH_ENABLED
#define RESET_BY_SWITCH_ENABLED 1
#endif

// 用于模拟微动开关触发的编码器阈值（当角度 <= 此值时认为触发）
#ifndef RESET_SWITCH_ANGLE_THRESHOLD
#define RESET_SWITCH_ANGLE_THRESHOLD 2
#endif
