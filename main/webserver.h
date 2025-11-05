#ifndef WEBSERVER_H
#define WEBSERVER_H

#include <stdint.h>

// Web Server 模块头文件

// 初始化 Wi-Fi AP 模式和 HTTP Server
void webserver_init(void);

// 停止 Web Server（若需要）
void webserver_stop(void);

// 更新网页端滑块位置（在非手动模式下调用，同步实际值到前端）
void webserver_update_slider_values(int16_t rotation_speed, int16_t position);

#endif // WEBSERVER_H