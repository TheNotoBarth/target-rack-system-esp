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
