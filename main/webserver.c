#include "webserver.h"
#include "config.h"
#include "serial_cboard.h"
#include "ui_state.h"
#include "display_uart.h"
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_http_server.h"

static const char *TAG = "webserver";

// Wi-Fi AP 配置
#define WIFI_SSID      "RM_Target"
#define WIFI_PASS      "12345678"
#define WIFI_CHANNEL   1
#define MAX_STA_CONN   4

// HTTP Server 句柄
static httpd_handle_t server = NULL;

// 滑块当前值（用于非手动模式下的同步）
static int16_t s_rotation_value = 0;
static int16_t s_position_value = 0;
static SemaphoreHandle_t s_slider_lock = NULL;

// 更新滑块值（线程安全）
void webserver_update_slider_values(int16_t rotation_speed, int16_t position)
{
	if (s_slider_lock) xSemaphoreTake(s_slider_lock, portMAX_DELAY);
	s_rotation_value = rotation_speed;
	s_position_value = position;
	if (s_slider_lock) xSemaphoreGive(s_slider_lock);
}

// HTML 页面内容（嵌入式）
static const char html_page[] = 
"<!DOCTYPE html>\n"
"<html>\n"
"<head>\n"
"    <meta charset='UTF-8'>\n"
"    <meta name='viewport' content='width=device-width, initial-scale=1.0'>\n"
"    <title>Target Rack Control</title>\n"
"    <style>\n"
"        body{font-family:Arial,sans-serif;margin:0;padding:20px;background:#1a1a1a;color:#fff;}\n"
"        h1{text-align:center;color:#4CAF50;}\n"
"        .container{max-width:800px;margin:0 auto;background:#2a2a2a;padding:20px;border-radius:10px;}\n"
"        .section{margin:20px 0;padding:15px;background:#333;border-radius:5px;}\n"
"        .section h2{margin-top:0;color:#4CAF50;}\n"
"        .status{display:flex;justify-content:space-between;margin:10px 0;}\n"
"        .status-item{flex:1;margin:0 10px;padding:10px;background:#444;border-radius:5px;}\n"
"        .status-label{color:#aaa;font-size:12px;}\n"
"        .status-value{font-size:20px;font-weight:bold;color:#4CAF50;}\n"
"        .control{margin:15px 0;}\n"
"        .control label{display:block;margin-bottom:5px;color:#aaa;}\n"
"        .slider{width:100%;height:30px;}\n"
"        .buttons{display:flex;gap:10px;margin:15px 0;justify-content:center;}\n"
"        .btn{padding:20px 40px;font-size:18px;border:none;border-radius:5px;cursor:pointer;background:#4CAF50;color:white;transition:background 0.3s;}\n"
"        .btn:hover{background:#45a049;}\n"
"        .btn:active{background:#357a38;}\n"
"        .mode-indicator{text-align:center;padding:10px;background:#555;border-radius:5px;font-size:18px;margin-bottom:20px;}\n"
"        .display-preview{background:#000;color:#0f0;padding:15px;border-radius:5px;font-family:monospace;font-size:14px;line-height:1.6;}\n"
"    </style>\n"
"</head>\n"
"<body>\n"
"    <div class='container'>\n"
"        <h1>🎯 Target Rack Control System</h1>\n"
"        <div class='mode-indicator'>Current Mode: <span id='mode'>Loading...</span></div>\n"
"\n"
"        <div class='section'>\n"
"            <h2>📺 Display Screen Preview</h2>\n"
"            <div class='display-preview' id='display_preview'>\n"
"                Loading...\n"
"            </div>\n"
"        </div>\n"
"\n"
"        <div class='section'>\n"
"            <h2>📊 Motor Status</h2>\n"
"            <div class='status'>\n"
"                <div class='status-item'>\n"
"                    <div class='status-label'>GM6020 Angle</div>\n"
"                    <div class='status-value'><span id='gm6020_angle'>0</span>°</div>\n"
"                </div>\n"
"                <div class='status-item'>\n"
"                    <div class='status-label'>GM6020 Speed</div>\n"
"                    <div class='status-value'><span id='gm6020_speed'>0</span> rpm</div>\n"
"                </div>\n"
"                <div class='status-item'>\n"
"                    <div class='status-label'>GM6020 Temp</div>\n"
"                    <div class='status-value'><span id='gm6020_temp'>0</span>°C</div>\n"
"                </div>\n"
"            </div>\n"
"            <div class='status'>\n"
"                <div class='status-item'>\n"
"                    <div class='status-label'>M3508 Position</div>\n"
"                    <div class='status-value'><span id='m3508_pos'>0</span></div>\n"
"                </div>\n"
"                <div class='status-item'>\n"
"                    <div class='status-label'>M3508 Speed</div>\n"
"                    <div class='status-value'><span id='m3508_speed'>0</span> rpm</div>\n"
"                </div>\n"
"                <div class='status-item'>\n"
"                    <div class='status-label'>M3508 Temp</div>\n"
"                    <div class='status-value'><span id='m3508_temp'>0</span>°C</div>\n"
"                </div>\n"
"            </div>\n"
"        </div>\n"
"\n"
"        <div class='section'>\n"
"            <h2>🎮 Manual Control</h2>\n"
"            <div class='control'>\n"
"                <label>Rotation Speed: <span id='rot_val'>0</span> rpm</label>\n"
"                <input type='range' class='slider' id='rotation' min='-100' max='100' value='0' step='1'>\n"
"            </div>\n"
"            <div class='control'>\n"
"                <label>Position: <span id='pos_val'>0</span></label>\n"
"                <input type='range' class='slider' id='position' min='0' max='8191' value='0' step='10'>\n"
"            </div>\n"
"        </div>\n"
"\n"
"        <div class='section'>\n"
"            <h2>⚙️ Virtual Buttons</h2>\n"
"            <div class='buttons'>\n"
"                <button class='btn' onclick='pressButton(\"up\")'>⬆️ UP</button>\n"
"                <button class='btn' onclick='pressButton(\"down\")'>⬇️ DOWN</button>\n"
"                <button class='btn' onclick='pressButton(\"ok\")'>✅ OK</button>\n"
"            </div>\n"
"        </div>\n"
"\n"
"        <div class='section'>\n"
"            <h2>📡 WiFi Info</h2>\n"
"            <div>SSID: <strong>RM_Target</strong></div>\n"
"            <div>Password: <strong>12345678</strong></div>\n"
"        </div>\n"
"    </div>\n"
"\n"
"    <script>\n"
"        let rotSlider = document.getElementById('rotation');\n"
"        let posSlider = document.getElementById('position');\n"
"        let rotVal = document.getElementById('rot_val');\n"
"        let posVal = document.getElementById('pos_val');\n"
"        let userInteracting = false;\n"
"\n"
"        rotSlider.oninput = function(){\n"
"            rotVal.textContent = this.value;\n"
"            userInteracting = true;\n"
"            fetch('/api/rotation?value=' + this.value);\n"
"        };\n"
"\n"
"        posSlider.oninput = function(){\n"
"            posVal.textContent = this.value;\n"
"            userInteracting = true;\n"
"            fetch('/api/position?value=' + this.value);\n"
"        };\n"
"\n"
"        function pressButton(btn){\n"
"            fetch('/api/button?btn=' + btn)\n"
"            .then(r => r.json())\n"
"            .then(d => console.log('Button press:', d));\n"
"        }\n"
"\n"
"        function updateStatus(){\n"
"            fetch('/api/status')\n"
"            .then(r => r.json())\n"
"            .then(d => {\n"
"                document.getElementById('mode').textContent = d.mode;\n"
"                document.getElementById('gm6020_angle').textContent = d.gm6020.angle.toFixed(1);\n"
"                document.getElementById('gm6020_speed').textContent = d.gm6020.speed.toFixed(1);\n"
"                document.getElementById('gm6020_temp').textContent = d.gm6020.temp;\n"
"                document.getElementById('m3508_pos').textContent = d.m3508.position;\n"
"                document.getElementById('m3508_speed').textContent = d.m3508.speed.toFixed(1);\n"
"                document.getElementById('m3508_temp').textContent = d.m3508.temp;\n"
"\n"
"                // 更新串口屏预览\n"
"                let preview = 'Mode: ' + d.mode + '\\n';\n"
"                preview += 'M1 A:' + d.gm6020.angle.toFixed(0) + '  S:' + d.gm6020.speed.toFixed(0) + '\\n';\n"
"                preview += '   I:' + d.gm6020.current + '  T:' + d.gm6020.temp + 'C\\n';\n"
"                preview += 'M2 A:' + d.m3508.position + '  S:' + d.m3508.speed.toFixed(0) + '\\n';\n"
"                preview += '   I:' + d.m3508.current + '  T:' + d.m3508.temp + 'C';\n"
"                document.getElementById('display_preview').textContent = preview;\n"
"\n"
"                // 非手动模式下，同步滑块位置（避免用户正在操作时更新）\n"
"                if (d.mode !== 'MANUAL' && !userInteracting) {\n"
"                    rotSlider.value = d.slider_rotation;\n"
"                    rotVal.textContent = d.slider_rotation;\n"
"                    posSlider.value = d.slider_position;\n"
"                    posVal.textContent = d.slider_position;\n"
"                }\n"
"            })\n"
"            .catch(err => {\n"
"                document.getElementById('mode').textContent = 'Error';\n"
"            });\n"
"            // 重置交互标志\n"
"            setTimeout(() => { userInteracting = false; }, 500);\n"
"        }\n"
"\n"
"        setInterval(updateStatus, 200);\n"
"        updateStatus();\n"
"    </script>\n"
"</body>\n"
"</html>\n";

// HTTP 处理函数：根页面
static esp_err_t root_handler(httpd_req_t *req)
{
	httpd_resp_set_type(req, "text/html");
	httpd_resp_send(req, html_page, HTTPD_RESP_USE_STRLEN);
	return ESP_OK;
}

// HTTP 处理函数：/api/status - 返回电机状态与模式信息（JSON）
static esp_err_t status_handler(httpd_req_t *req)
{
	const motor_status_t *m1 = get_motor_status(1);
	const motor_status_t *m2 = get_motor_status(2);
	control_mode_t mode = ui_state_get_mode();
	const char *mode_names[] = {"MANUAL", "PRESET1", "PRESET2"};
	const char *mode_str = (mode < MODE_COUNT) ? mode_names[mode] : "UNKNOWN";

	// 获取滑块值
	int16_t rot_val = 0, pos_val = 0;
	if (s_slider_lock) xSemaphoreTake(s_slider_lock, portMAX_DELAY);
	rot_val = s_rotation_value;
	pos_val = s_position_value;
	if (s_slider_lock) xSemaphoreGive(s_slider_lock);

	char buf[512];
	int n = snprintf(buf, sizeof(buf),
		"{"
		"\"mode\":\"%s\","
		"\"gm6020\":{\"angle\":%.1f,\"speed\":%.1f,\"current\":%d,\"temp\":%u},"
		"\"m3508\":{\"position\":%u,\"speed\":%.1f,\"current\":%d,\"temp\":%u},"
		"\"slider_rotation\":%d,"
		"\"slider_position\":%d"
		"}",
		mode_str,
		m1 ? (m1->angle * 360.0f / 8191.0f) : 0.0f,
		m1 ? (float)m1->speed : 0.0f,
		m1 ? (int)m1->current : 0,
		m1 ? (unsigned)m1->temperature : 0u,
		m2 ? (unsigned)m2->angle : 0u,
		m2 ? (float)m2->speed : 0.0f,
		m2 ? (int)m2->current : 0,
		m2 ? (unsigned)m2->temperature : 0u,
		(int)rot_val,
		(int)pos_val
	);

	httpd_resp_set_type(req, "application/json");
	httpd_resp_send(req, buf, n);
	return ESP_OK;
}

// HTTP 处理函数：/api/rotation - 设置旋转速度
static esp_err_t rotation_handler(httpd_req_t *req)
{
	char buf[64];
	size_t buf_len = httpd_req_get_url_query_len(req) + 1;
	if (buf_len > 1 && buf_len <= sizeof(buf)) {
		if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
			char value_str[16];
			if (httpd_query_key_value(buf, "value", value_str, sizeof(value_str)) == ESP_OK) {
				int value = atoi(value_str);
				// 切换到手动模式
				if (ui_state_get_mode() != MODE_MANUAL) {
					ui_state_set_mode(MODE_MANUAL);
				}
				// 发送速度控制命令到电机1 (GM6020)
				send_motor_command(1, (int16_t)value, 0, 0);
				// 更新滑块值
				webserver_update_slider_values((int16_t)value, s_position_value);
				ESP_LOGI(TAG, "Set rotation speed: %d", value);
			}
		}
	}
	httpd_resp_set_type(req, "application/json");
	httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
	return ESP_OK;
}

// HTTP 处理函数：/api/position - 设置位置
static esp_err_t position_handler(httpd_req_t *req)
{
	char buf[64];
	size_t buf_len = httpd_req_get_url_query_len(req) + 1;
	if (buf_len > 1 && buf_len <= sizeof(buf)) {
		if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
			char value_str[16];
			if (httpd_query_key_value(buf, "value", value_str, sizeof(value_str)) == ESP_OK) {
				int value = atoi(value_str);
				// 切换到手动模式
				if (ui_state_get_mode() != MODE_MANUAL) {
					ui_state_set_mode(MODE_MANUAL);
				}
				// 发送位置控制命令到电机2 (M3508)
				send_motor_command(2, 0, (int16_t)value, 1);
				// 更新滑块值
				webserver_update_slider_values(s_rotation_value, (int16_t)value);
				ESP_LOGI(TAG, "Set position: %d", value);
			}
		}
	}
	httpd_resp_set_type(req, "application/json");
	httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
	return ESP_OK;
}

// HTTP 处理函数：/api/button - 虚拟按键
static esp_err_t button_handler(httpd_req_t *req)
{
	char buf[64];
	size_t buf_len = httpd_req_get_url_query_len(req) + 1;
	if (buf_len > 1 && buf_len <= sizeof(buf)) {
		if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
			char btn_str[16];
			if (httpd_query_key_value(buf, "btn", btn_str, sizeof(btn_str)) == ESP_OK) {
				if (strcmp(btn_str, "up") == 0) {
					ui_state_button_event_up();
					ESP_LOGI(TAG, "Virtual button: UP");
				} else if (strcmp(btn_str, "down") == 0) {
					ui_state_button_event_down();
					ESP_LOGI(TAG, "Virtual button: DOWN");
				} else if (strcmp(btn_str, "ok") == 0) {
					ui_state_button_event_ok();
					ESP_LOGI(TAG, "Virtual button: OK");
				}
			}
		}
	}
	httpd_resp_set_type(req, "application/json");
	httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
	return ESP_OK;
}

// Wi-Fi 事件处理器
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
	if (event_id == WIFI_EVENT_AP_STACONNECTED) {
		wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
		ESP_LOGI(TAG, "Station %02x:%02x:%02x:%02x:%02x:%02x joined, AID=%d",
				 event->mac[0], event->mac[1], event->mac[2],
				 event->mac[3], event->mac[4], event->mac[5], event->aid);
	} else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
		wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
		ESP_LOGI(TAG, "Station %02x:%02x:%02x:%02x:%02x:%02x left, AID=%d",
				 event->mac[0], event->mac[1], event->mac[2],
				 event->mac[3], event->mac[4], event->mac[5], event->aid);
	}
}

// 启动 HTTP Server
static httpd_handle_t start_webserver(void)
{
	httpd_config_t config = HTTPD_DEFAULT_CONFIG();
	config.lru_purge_enable = true;

	httpd_handle_t srv = NULL;
	if (httpd_start(&srv, &config) == ESP_OK) {
		// 注册 URI 处理函数
		httpd_uri_t root_uri = {
			.uri       = "/",
			.method    = HTTP_GET,
			.handler   = root_handler,
			.user_ctx  = NULL
		};
		httpd_register_uri_handler(srv, &root_uri);

		httpd_uri_t status_uri = {
			.uri       = "/api/status",
			.method    = HTTP_GET,
			.handler   = status_handler,
			.user_ctx  = NULL
		};
		httpd_register_uri_handler(srv, &status_uri);

		httpd_uri_t rotation_uri = {
			.uri       = "/api/rotation",
			.method    = HTTP_GET,
			.handler   = rotation_handler,
			.user_ctx  = NULL
		};
		httpd_register_uri_handler(srv, &rotation_uri);

		httpd_uri_t position_uri = {
			.uri       = "/api/position",
			.method    = HTTP_GET,
			.handler   = position_handler,
			.user_ctx  = NULL
		};
		httpd_register_uri_handler(srv, &position_uri);

		httpd_uri_t button_uri = {
			.uri       = "/api/button",
			.method    = HTTP_GET,
			.handler   = button_handler,
			.user_ctx  = NULL
		};
		httpd_register_uri_handler(srv, &button_uri);

		ESP_LOGI(TAG, "HTTP server started");
		return srv;
	}

	ESP_LOGE(TAG, "Error starting HTTP server!");
	return NULL;
}

// 初始化 Wi-Fi AP
static void wifi_init_softap(void)
{
	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	esp_netif_create_default_wifi_ap();

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));

	ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
														ESP_EVENT_ANY_ID,
														&wifi_event_handler,
														NULL,
														NULL));

	wifi_config_t wifi_config = {
		.ap = {
			.ssid = WIFI_SSID,
			.ssid_len = strlen(WIFI_SSID),
			.channel = WIFI_CHANNEL,
			.password = WIFI_PASS,
			.max_connection = MAX_STA_CONN,
			.authmode = WIFI_AUTH_WPA_WPA2_PSK
		},
	};

	if (strlen(WIFI_PASS) == 0) {
		wifi_config.ap.authmode = WIFI_AUTH_OPEN;
	}

	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
	ESP_ERROR_CHECK(esp_wifi_start());

	ESP_LOGI(TAG, "Wi-Fi AP started. SSID:%s password:%s channel:%d",
			 WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);
}

// 后台任务：在非手动模式下更新滑块值
static void slider_sync_task(void *arg)
{
	while (1) {
		control_mode_t mode = ui_state_get_mode();
		if (mode != MODE_MANUAL) {
			// 同步实际电机值到滑块
			const motor_status_t *m1 = get_motor_status(1);
			const motor_status_t *m2 = get_motor_status(2);
			if (m1 && m2) {
				webserver_update_slider_values(m1->speed, m2->angle);
			}
		}
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

void webserver_init(void)
{
	// 初始化 NVS
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	// 创建滑块锁
	s_slider_lock = xSemaphoreCreateMutex();

	// 初始化 Wi-Fi AP
	wifi_init_softap();

	// 启动 HTTP Server
	server = start_webserver();

	// 启动滑块同步任务
	xTaskCreate(slider_sync_task, "slider_sync", 2048, NULL, 5, NULL);

	ESP_LOGI(TAG, "Web server initialized. Connect to Wi-Fi AP and visit http://192.168.4.1");
}

void webserver_stop(void)
{
	if (server) {
		httpd_stop(server);
		server = NULL;
		ESP_LOGI(TAG, "HTTP server stopped");
	}
}