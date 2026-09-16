/**
 * ESP32-S3 继电器控制端 (WiFiManager配网版)
 *
 * 配网模式: AP "ESP32_Setup" (开放) + Captive Portal (192.168.4.1)
 * 正常模式: AP+STA, AP("ESP32_Setup"), STA连接家WiFi
 * ESP-NOW发送指令给C3
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "math.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "esp_http_client.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "lwip/sockets.h"
#include "lwip/ip4_addr.h"
#include "lwip/inet.h"

static const char *TAG = "RELAY_S3";

#define WIFI_CONNECTED_BIT BIT0
static EventGroupHandle_t wifi_event_group;

/* C3 的 MAC 地址 (初始硬编码, 运行时动态学习更新) */
static uint8_t c3_mac[6] = {0x10, 0x00, 0x3B, 0xCE, 0xF9, 0x35};
static uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/* 状态 */
static bool sta_connected = false;
static char sta_ip[16] = "0.0.0.0";
static char sta_ssid[33] = "";
static bool is_config_mode = false;

/* 继电器状态 (由 C3 回传) */
static int relay_on = 0;
static int relay_active_level = 1;
static bool relay_status_known = false;

/* C3 的 IP 地址 (通过 UDP 广播发现) */
static char c3_ip[32] = "";
static volatile bool c3_ip_known = false;

/* ESP-NOW 通信确认标志 */
static volatile bool status_received = false;     /* C3 回传状态确认 */
static uint8_t real_c3_mac[6] = {0};              /* 动态学到的 C3 真实 MAC */
static volatile bool c3_mac_learned = false;      /* 是否已学到真实 MAC */

/* ========== HC-SR04 超声波传感器 ========== */

/* 前向声明 */
static esp_err_t send_cmd(char cmd);

#define TRIG_GPIO       5
#define ECHO_GPIO       18
#define SAMPLE_COUNT    40       /* 每次采集40次（高精度） */
#define SAMPLE_INTERVAL_MS 1    /* 采样间隔1ms（极限提速） */
#define LOOP_INTERVAL_MS 1      /* 主循环间隔1ms */
#define ECHO_TIMEOUT_US 8000    /* 8ms 超时（近距离<70cm足够） */
#define MAX_DISTANCE_CM 400
#define MIN_VALID_CM    1.0
#define AUTO_OFF_DEFAULT 2.5
#define AUTO_ON_DEFAULT  10.0
static volatile float auto_off_cm = AUTO_OFF_DEFAULT;  /* 断开阈值, 可通过HTTP修改 */
static volatile float auto_on_cm = AUTO_ON_DEFAULT;    /* 吸合阈值, 可通过HTTP修改 */
#define DIST_MAX_DISPLAY 20.0   /* 进度条最大显示距离 */
#define OUTLIER_THRESHOLD 8.0   /* 异常值阈值: 偏差>8cm 丢弃 */
#define TRIM_COUNT      7       /* 去极值: 去掉最大最小各7个，中间26个取平均 */

/* 自动/手动模式 */
static volatile bool auto_mode = false;
static volatile float current_distance = -1.0;  /* 滤波后距离, -1=无效 */

/* 卡尔曼滤波器 (1D) */
typedef struct {
    float x;  /* 估计值 */
    float P;  /* 误差协方差 */
    float Q;  /* 过程噪声 */
    float R;  /* 测量噪声 */
} kalman_t;

static kalman_t dist_kalman = {.x = 10.0, .P = 1.0, .Q = 0.3, .R = 0.8};

static float kalman_update(kalman_t *k, float meas)
{
    k->P = k->P + k->Q;
    float K = k->P / (k->P + k->R);
    k->x = k->x + K * (meas - k->x);
    k->P = (1.0f - K) * k->P;
    return k->x;
}

/* HC-SR04 单次测距 */
static float hc_sr04_read_once(void)
{
    /* 发送 10us 触发脉冲 */
    gpio_set_level(TRIG_GPIO, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG_GPIO, 0);

    /* 等待 Echo 上升沿 */
    int64_t t0 = esp_timer_get_time();
    while (gpio_get_level(ECHO_GPIO) == 0) {
        if (esp_timer_get_time() - t0 > ECHO_TIMEOUT_US) return -1;
    }
    int64_t echo_start = esp_timer_get_time();

    /* 测量 Echo 高电平持续时间 */
    while (gpio_get_level(ECHO_GPIO) == 1) {
        if (esp_timer_get_time() - echo_start > ECHO_TIMEOUT_US) return -1;
    }
    int64_t echo_end = esp_timer_get_time();

    /* 距离(cm) = us / 58 */
    float dist = (float)(echo_end - echo_start) / 58.0f;
    if (dist < MIN_VALID_CM || dist > MAX_DISTANCE_CM) return -1;
    return dist;
}

/* 采集40次 + 异常剔除 + 去极值中值 + 卡尔曼三重滤波 */
static float read_distance_filtered(void)
{
    float samples[SAMPLE_COUNT];
    int valid = 0;
    float last_valid = current_distance;

    for (int i = 0; i < SAMPLE_COUNT; i++) {
        float d = hc_sr04_read_once();
        if (d > 0) {
            /* 第一重: 异常值剔除 */
            if (last_valid > 0 && fabsf(d - last_valid) > OUTLIER_THRESHOLD) {
                /* 丢弃 */
            } else {
                samples[valid++] = d;
                last_valid = d;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
    }

    if (valid < 3) return -1;

    /* 排序 */
    for (int i = 0; i < valid - 1; i++) {
        for (int j = 0; j < valid - i - 1; j++) {
            if (samples[j] > samples[j+1]) {
                float t = samples[j]; samples[j] = samples[j+1]; samples[j+1] = t;
            }
        }
    }

    /* 第二重: 去极值中值（去掉最大最小各TRIM_COUNT个，中间取平均） */
    float trimmed_avg;
    int remain = valid - 2 * TRIM_COUNT;
    if (remain > 0) {
        float sum = 0;
        for (int i = TRIM_COUNT; i < valid - TRIM_COUNT; i++) {
            sum += samples[i];
        }
        trimmed_avg = sum / remain;
    } else {
        /* 有效值不足，用中值 */
        trimmed_avg = samples[valid / 2];
    }

    /* 第三重: 卡尔曼平滑 */
    return kalman_update(&dist_kalman, trimmed_avg);
}

/* 超声波采集 + 自动模式控制任务 */
static void ultrasonic_task(void *pv)
{
    ESP_LOGI(TAG, "超声波任务启动: Trig=GPIO%d, Echo=GPIO%d", TRIG_GPIO, ECHO_GPIO);

    int64_t last_cmd_time = 0;     /* 上次发送指令的时间(us) */
    char last_auto_cmd = 0;        /* 上次自动模式发送的指令 */

    while (1) {
        float dist = read_distance_filtered();
        current_distance = dist;

        if (dist > 0) {
            ESP_LOGI(TAG, "距离: %.2f cm (模式: %s)", dist, auto_mode ? "自动" : "手动");

            /* 自动模式: 根据距离控制继电器（不检查 relay_on，用时间间隔防重复） */
            if (auto_mode) {
                int64_t now = esp_timer_get_time();
                /* 距离 < 2.5cm → 断开（100ms 内不重发相同指令） */
                if (dist < auto_off_cm) {
                    if (last_auto_cmd != '0' || (now - last_cmd_time) > 100000) {
                        ESP_LOGI(TAG, "自动: 距离<%.1fcm → 断开", auto_off_cm);
                        send_cmd('0');
                        last_auto_cmd = '0';
                        last_cmd_time = now;
                    }
                } else if (dist > auto_on_cm) {
                    if (last_auto_cmd != '1' || (now - last_cmd_time) > 100000) {
                        ESP_LOGI(TAG, "自动: 距离>%.1fcm → 吸合", auto_on_cm);
                        send_cmd('1');
                        last_auto_cmd = '1';
                        last_cmd_time = now;
                    }
                }
            }
        } else {
            ESP_LOGW(TAG, "距离读取无效");
        }
        vTaskDelay(pdMS_TO_TICKS(LOOP_INTERVAL_MS));  /* 1ms 循环间隔 */
    }
}

/* ========== NVS 配置存储 ========== */
static esp_err_t nvs_save_wifi(const char *ssid, const char *pass)
{
    nvs_handle_t h;
    esp_err_t r = nvs_open("wifi", NVS_READWRITE, &h);
    if (r != ESP_OK) return r;
    nvs_set_str(h, "ssid", ssid);
    nvs_set_str(h, "pass", pass);
    nvs_commit(h);
    nvs_close(h);
    return ESP_OK;
}

static esp_err_t nvs_load_wifi(char *ssid, size_t ssid_len, char *pass, size_t pass_len)
{
    nvs_handle_t h;
    esp_err_t r = nvs_open("wifi", NVS_READONLY, &h);
    if (r != ESP_OK) return r;
    r = nvs_get_str(h, "ssid", ssid, &ssid_len);
    if (r == ESP_OK) r = nvs_get_str(h, "pass", pass, &pass_len);
    nvs_close(h);
    return r;
}

/* ========== 手动 JSON 解析 ========== */
/* 从JSON字符串中提取字段值 (简单实现, 不处理转义) */
static void json_extract(const char *json, const char *key, char *out, size_t out_len)
{
    out[0] = 0;
    /* 构造搜索键: "key":"  */
    char search[64];
    snprintf(search, sizeof(search), "\"%s\":\"", key);
    const char *p = strstr(json, search);
    if (!p) return;
    p += strlen(search);
    /* 找到下一个引号 */
    const char *end = strchr(p, '"');
    if (!end) return;
    size_t len = end - p;
    if (len >= out_len) len = out_len - 1;
    memcpy(out, p, len);
    out[len] = 0;
}

/* ========== ESP-NOW 发送 ========== */

/* 更新 C3 peer MAC（在非回调线程中调用） */
static void update_c3_peer(void)
{
    if (!c3_mac_learned) return;
    if (memcmp(c3_mac, real_c3_mac, 6) == 0) return;  /* 没变化 */

    esp_now_del_peer(c3_mac);  /* 删除旧 peer */
    memcpy(c3_mac, real_c3_mac, 6);  /* 更新为真实 MAC */
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, c3_mac, 6);
    peer.channel = 0;
    peer.ifidx = WIFI_IF_AP;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
    ESP_LOGI(TAG, "Updated C3 peer MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             c3_mac[0],c3_mac[1],c3_mac[2],c3_mac[3],c3_mac[4],c3_mac[5]);
}

/* ========== HTTP 客户端 (S3 → C3) ========== */
/* 通过 HTTP 控制 C3 继电器，完全绕过 ESP-NOW */

static esp_err_t http_send_to_c3(char cmd)
{
    if (!c3_ip_known || strlen(c3_ip) == 0) {
        ESP_LOGW(TAG, "C3 IP 未知，无法 HTTP 控制");
        return ESP_FAIL;
    }

    char url[64];
    snprintf(url, sizeof(url), "http://%s/cmd?c=%c", c3_ip, cmd);

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .timeout_ms = 2000,
        .disable_auto_redirect = true,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_perform(client);

    int status_code = 0;
    if (err == ESP_OK) {
        status_code = esp_http_client_get_status_code(client);
    }
    esp_http_client_cleanup(client);

    bool ok = (err == ESP_OK && status_code == 200);
    ESP_LOGI(TAG, "HTTP → C3 '%c' : %s (status=%d)", cmd, ok?"OK":"FAIL", status_code);

    if (ok) {
        if (cmd == '1')      relay_on = 1;
        else if (cmd == '0') relay_on = 0;
        else if (cmd == 'T') relay_on = !relay_on;
        relay_status_known = true;
        return ESP_OK;
    }
    return ESP_FAIL;
}

/* UDP 监听任务: 接收 C3 广播的 IP */
static void udp_listen_task(void *pv)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) { vTaskDelete(NULL); return; }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(3333);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "UDP listen bind failed");
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "UDP 监听启动 (port 3333), 等待 C3 广播...");

    char buf[64];
    struct sockaddr_in src;
    socklen_t slen = sizeof(src);

    while (1) {
        int len = recvfrom(sock, buf, sizeof(buf)-1, 0, (struct sockaddr *)&src, &slen);
        if (len > 0) {
            buf[len] = 0;
            /* C3 广播格式: "C3_IP:xxx.xxx.xxx.xxx" */
            if (strncmp(buf, "C3_IP:", 6) == 0) {
                char *ip = buf + 6;
                /* 去除可能的换行 */
                char *nl = strchr(ip, '\n');
                if (nl) *nl = 0;
                nl = strchr(ip, '\r');
                if (nl) *nl = 0;

                if (!c3_ip_known || strcmp(c3_ip, ip) != 0) {
                    strncpy(c3_ip, ip, sizeof(c3_ip)-1);
                    c3_ip[sizeof(c3_ip)-1] = 0;
                    c3_ip_known = true;
                    ESP_LOGI(TAG, "发现 C3 IP: %s (from %s)", c3_ip, inet_ntoa(src.sin_addr));
                }
            }
        }
    }
    close(sock);
    vTaskDelete(NULL);
}

static esp_err_t send_cmd(char cmd)
{
    /* 优先用 HTTP 控制 C3（如果 C3 有 IP） */
    if (c3_ip_known) {
        esp_err_t ret = http_send_to_c3(cmd);
        if (ret == ESP_OK) return ESP_OK;
        ESP_LOGW(TAG, "HTTP 控制 C3 失败，尝试 ESP-NOW");
    }

    /* ESP-NOW 控制：广播发送 5 次，不检查返回值（WiFi 驱动会缓存稍后发送） */
    uint8_t data = (uint8_t)cmd;

    for (int i = 0; i < 5; i++) {
        esp_err_t ret = esp_now_send(broadcast_mac, &data, 1);
        ESP_LOGI(TAG, "ESP-NOW broadcast '%c' attempt %d -> %s", cmd, i+1, ret==ESP_OK?"OK":"FAIL");
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    /* 始终乐观更新本地状态（C3 总是能收到广播指令） */
    if (cmd == '1')      relay_on = 1;
    else if (cmd == '0') relay_on = 0;
    else if (cmd == 'T') relay_on = !relay_on;
    relay_status_known = true;
    return ESP_OK;
}

static void espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    /* 只记录日志，不依赖 MAC 层 ACK */
    ESP_LOGI(TAG, "ESP-NOW tx: %s", status==ESP_NOW_SEND_SUCCESS?"SUCCESS":"FAIL");
}

/* C3 回传状态: [relay_on, relay_active_level] */
static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    /* 动态学习 C3 真实 MAC（回调中只记录，不调用 ESP-NOW API） */
    if (info && info->src_addr && !c3_mac_learned) {
        memcpy(real_c3_mac, info->src_addr, 6);
        c3_mac_learned = true;
        ESP_LOGI(TAG, "Learned C3 MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                 real_c3_mac[0],real_c3_mac[1],real_c3_mac[2],
                 real_c3_mac[3],real_c3_mac[4],real_c3_mac[5]);
    }

    if (len >= 2) {
        relay_on = data[0];
        relay_active_level = data[1];
        relay_status_known = true;
    }
    status_received = true;  /* 标记收到 C3 回传 */
    ESP_LOGI(TAG, "ESP-NOW recv: on=%d level=%d", relay_on, relay_active_level);
}

/* ========== DNS 服务器 (Captive Portal) ========== */
static void dns_server_task(void *pv)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) { vTaskDelete(NULL); return; }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(53);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "DNS server started (captive portal)");
    uint8_t buf[128];
    while (1) {
        struct sockaddr_in src;
        socklen_t src_len = sizeof(src);
        int len = recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr*)&src, &src_len);
        if (len < 31) continue;

        /* 构建DNS响应: 所有域名解析为 192.168.4.1 */
        buf[2] |= 0x80;
        buf[3] |= 0x80;
        buf[7] = 1;

        int pos = len;
        buf[pos++] = 0xC0; buf[pos++] = 0x0C;
        buf[pos++] = 0x00; buf[pos++] = 0x01;
        buf[pos++] = 0x00; buf[pos++] = 0x01;
        buf[pos++] = 0x00; buf[pos++] = 0x00; buf[pos++] = 0x00; buf[pos++] = 0x3C;
        buf[pos++] = 0x00; buf[pos++] = 0x04;
        buf[pos++] = 192;  buf[pos++] = 168;  buf[pos++] = 4;    buf[pos++] = 1;

        sendto(sock, buf, pos, 0, (struct sockaddr*)&src, src_len);
    }
    close(sock);
    vTaskDelete(NULL);
}

/* ========== UDP 设备发现服务 ========== */
#define DISCOVERY_PORT 18080
#define DISCOVERY_MAGIC "RELAY_DISCOVER"

static void udp_discovery_task(void *pv)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) { vTaskDelete(NULL); return; }

    int broadcast = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(DISCOVERY_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "UDP discovery bind failed");
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "UDP discovery listening on port %d", DISCOVERY_PORT);
    uint8_t buf[128];
    while (1) {
        struct sockaddr_in src;
        socklen_t src_len = sizeof(src);
        int len = recvfrom(sock, buf, sizeof(buf) - 1, 0,
                           (struct sockaddr*)&src, &src_len);
        if (len <= 0) continue;
        buf[len] = 0;

        if (strstr((char*)buf, DISCOVERY_MAGIC) == NULL) continue;

        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_WIFI_STA);
        char resp[192];
        snprintf(resp, sizeof(resp),
            "{\"name\":\"RELAY-S3\",\"ip\":\"%s\",\"mac\":\"%02X:%02X:%02X:%02X:%02X:%02X\","
            "\"sta\":%d,\"relay_on\":%d}",
            sta_ip, mac[0],mac[1],mac[2],mac[3],mac[4],mac[5],
            sta_connected?1:0, relay_on);

        sendto(sock, resp, strlen(resp), 0,
               (struct sockaddr*)&src, src_len);
        /* v6.0.2: sin_addr 是 BSD struct in_addr (字段 s_addr), 不能直接传给 IP2STR */
        char src_ip[16];
        inet_ntop(AF_INET, &src.sin_addr, src_ip, sizeof(src_ip));
        ESP_LOGI(TAG, "Discovery reply -> %s", src_ip);
    }
    close(sock);
    vTaskDelete(NULL);
}

/* ========== HTTP 接口 ========== */

/* 配网页面 */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    const char *html =
    "<!DOCTYPE html><html><head><meta charset='UTF-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>ESP32 配网</title>"
    "<style>"
    "body{font-family:sans-serif;background:#0F172A;color:#F1F5F9;"
    "display:flex;justify-content:center;align-items:center;min-height:100vh;margin:0}"
    ".card{background:#1E293B;padding:30px;border-radius:16px;"
    "box-shadow:0 4px 20px rgba(0,0,0,0.3);width:90%;max-width:360px}"
    "h2{text-align:center;color:#22D3EE;margin:0 0 20px}"
    "label{display:block;margin:10px 0 4px;font-size:14px;color:#94A3B8}"
    "input{width:100%;padding:10px;border:1px solid #334155;border-radius:8px;"
    "background:#0F172A;color:#F1F5F9;box-sizing:border-box;font-size:14px}"
    "button{width:100%;padding:12px;border:none;border-radius:8px;"
    "background:linear-gradient(135deg,#06B6D4,#0891B2);color:white;"
    "font-size:16px;cursor:pointer;margin-top:16px}"
    "button:active{opacity:0.8}"
    ".status{text-align:center;margin-top:12px;font-size:13px;color:#94A3B8}"
    "</style></head><body><div class='card'>"
    "<h2>ESP32 配网</h2>"
    "<form id='f'>"
    "<label>WiFi名称</label>"
    "<input id='s' placeholder='输入WiFi名称'>"
    "<label>WiFi密码</label>"
    "<input id='p' type='password' placeholder='输入WiFi密码'>"
    "<button type='button' onclick='submit()'>保存并连接</button>"
    "</form>"
    "<div id='st' class='status'></div>"
    "</div><script>"
    "function submit(){"
    "var s=document.getElementById('s').value;"
    "var p=document.getElementById('p').value;"
    "if(!s){alert('请输入WiFi名称');return;}"
    "document.getElementById('st').innerText='正在保存...';"
    "fetch('/config',{method:'POST',headers:{'Content-Type':'application/json'},"
    "body:JSON.stringify({ssid:s,password:p})})"
    ".then(r=>r.text()).then(t=>{"
    "if(t.includes('OK')){document.getElementById('st').innerText="
    "'保存成功，设备正在重启并连接WiFi...';}"
    "else{document.getElementById('st').innerText='保存失败: '+t;}"
    "}).catch(e=>{document.getElementById('st').innerText='错误: '+e;});}"
    "</script></body></html>";
    httpd_resp_sendstr(req, html);
    return ESP_OK;
}

/* /config 接口: 接受 JSON body 和 query params */
static esp_err_t config_post_handler(httpd_req_t *req)
{
    char buf[512] = {0};
    char ssid[33] = {0}, pass[65] = {0};
    bool parsed = false;

    /* 优先解析 JSON body */
    int body_len = req->content_len;
    if (body_len > 0 && body_len < (int)sizeof(buf)) {
        int ret = httpd_req_recv(req, buf, body_len);
        if (ret > 0) {
            buf[ret] = 0;
            json_extract(buf, "ssid", ssid, sizeof(ssid));
            json_extract(buf, "password", pass, sizeof(pass));
            if (ssid[0]) parsed = true;
        }
    }

    /* 回退: 解析 query params */
    if (!parsed) {
        int qlen = httpd_req_get_url_query_len(req);
        if (qlen > 0 && qlen < (int)sizeof(buf)) {
            httpd_req_get_url_query_str(req, buf, sizeof(buf));
            char *p = buf;
            while (*p) {
                if (strncmp(p, "ssid=", 5) == 0) {
                    char *v = p + 5;
                    char *amp = strchr(v, '&');
                    int vl = amp ? (int)(amp - v) : (int)strlen(v);
                    if (vl > 0 && vl < 33) { memcpy(ssid, v, vl); ssid[vl] = 0; parsed = true; }
                } else if (strncmp(p, "password=", 9) == 0) {
                    char *v = p + 9;
                    char *amp = strchr(v, '&');
                    int vl = amp ? (int)(amp - v) : (int)strlen(v);
                    if (vl > 0 && vl < 65) { memcpy(pass, v, vl); pass[vl] = 0; }
                }
                p++;
            }
        }
    }

    if (!parsed || ssid[0] == 0) {
        httpd_resp_sendstr(req, "ERR: no ssid");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Config received: ssid='%s'", ssid);
    nvs_save_wifi(ssid, pass);
    httpd_resp_sendstr(req, "OK");

    /* 1秒后重启 */
    esp_timer_handle_t reboot_timer;
    static esp_timer_create_args_t reboot_args;
    reboot_args.callback = (void(*)(void*))esp_restart;
    reboot_args.name = "reboot";
    esp_timer_create(&reboot_args, &reboot_timer);
    esp_timer_start_once(reboot_timer, 1000000);

    return ESP_OK;
}

/* /status 接口 */
static esp_err_t status_get_handler(httpd_req_t *req)
{
    char buf[256];
    snprintf(buf, sizeof(buf),
        "{\"mode\":\"%s\",\"ip\":\"%s\",\"connected\":%s,\"ssid\":\"%s\","
        "\"relay_on\":%d,\"relay_level\":%d,\"relay_known\":%s}",
        is_config_mode ? "config" : "sta",
        sta_ip,
        sta_connected ? "true" : "false",
        sta_ssid,
        relay_on, relay_active_level,
        relay_status_known ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

/* /scan 接口: ESP32 扫描周边 WiFi，返回 JSON 列表给 App 显示 */
static esp_err_t scan_get_handler(httpd_req_t *req)
{
    /* 在 APSTA 模式下用 STA 接口扫描 */
    wifi_scan_config_t scan_cfg = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
    };

    esp_err_t err = esp_wifi_scan_start(&scan_cfg, true);
    if (err != ESP_OK) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "[]");
        return ESP_OK;
    }

    uint16_t ap_count = 0;
    esp_wifi_scan_get_ap_num(&ap_count);
    if (ap_count == 0) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "[]");
        return ESP_OK;
    }

    wifi_ap_record_t *aps = malloc(ap_count * sizeof(wifi_ap_record_t));
    if (!aps) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "[]");
        return ESP_OK;
    }
    esp_wifi_scan_get_ap_records(&ap_count, aps);

    /* 扫描结果按 RSSI 降序，同 SSID 保留第一个(最强) */
    char *buf = malloc(4096);
    if (!buf) { free(aps); httpd_resp_set_type(req, "application/json"); httpd_resp_sendstr(req, "[]"); return ESP_OK; }
    int pos = 0;
    pos += sprintf(buf + pos, "[");

    int written = 0;
    for (int i = 0; i < ap_count && written < 20; i++) {
        /* 过滤空 SSID 和 ESP32_Setup 自身 */
        if (aps[i].ssid[0] == 0) continue;
        if (strcmp((char *)aps[i].ssid, "ESP32_Setup") == 0) continue;

        /* 去重: 同 SSID 跳过 (结果已按 RSSI 降序，第一个是最强的) */
        bool dup = false;
        for (int j = 0; j < i; j++) {
            if (strcmp((char *)aps[j].ssid, (char *)aps[i].ssid) == 0) {
                dup = true;
                break;
            }
        }
        if (dup) continue;

        int secure = (aps[i].authmode != WIFI_AUTH_OPEN) ? 1 : 0;
        if (written > 0) pos += sprintf(buf + pos, ",");
        pos += sprintf(buf + pos, "{\"ssid\":\"%s\",\"rssi\":%d,\"secure\":%d}",
                       (char *)aps[i].ssid, aps[i].rssi, secure);
        written++;
    }
    pos += sprintf(buf + pos, "]");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, pos);
    free(buf);
    free(aps);
    return ESP_OK;
}

/* /info 接口 */
static esp_err_t info_get_handler(httpd_req_t *req)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char buf[256];
    snprintf(buf, sizeof(buf),
        "{\"name\":\"RELAY-S3\",\"ip\":\"%s\",\"mac\":\"%02X:%02X:%02X:%02X:%02X:%02X\",\"sta\":%d}",
        sta_ip, mac[0],mac[1],mac[2],mac[3],mac[4],mac[5], sta_connected?1:0);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

/* /distance 接口: 返回距离、模式、继电器状态 */
static esp_err_t distance_get_handler(httpd_req_t *req)
{
    char buf[160];
    snprintf(buf, sizeof(buf),
        "{\"distance\":%.2f,\"mode\":\"%s\",\"relay_on\":%d,\"auto_off\":%.1f,\"auto_on\":%.1f}",
        current_distance,
        auto_mode ? "auto" : "manual",
        relay_on,
        auto_off_cm, auto_on_cm);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

/* /threshold 接口: GET /threshold?off=3.0&on=12.0 设置断开/吸合阈值 */
static esp_err_t threshold_get_handler(httpd_req_t *req)
{
    char buf[64] = {0};
    httpd_req_get_url_query_str(req, buf, sizeof(buf));
    float off = -1, on = -1;
    /* 解析 off= */
    char *p = strstr(buf, "off=");
    if (p) off = atof(p + 4);
    /* 解析 on= */
    p = strstr(buf, "on=");
    if (p) on = atof(p + 3);

    if (off >= 2.5f && off <= 6.0f) {
        auto_off_cm = off;
        ESP_LOGI(TAG, "断开阈值更新: %.1fcm", auto_off_cm);
    }
    if (on >= 10.0f && on <= 15.0f) {
        auto_on_cm = on;
        ESP_LOGI(TAG, "吸合阈值更新: %.1fcm", auto_on_cm);
    }
    ESP_LOGI(TAG, "/threshold: off=%.1f on=%.1f -> auto_off=%.1f auto_on=%.1f", off, on, auto_off_cm, auto_on_cm);
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

/* /mode 接口: POST mode=auto|manual 切换模式 */
static esp_err_t mode_post_handler(httpd_req_t *req)
{
    char buf[64] = {0};
    int total = req->content_len;
    int received = 0;
    /* 循环读取确保收完整 */
    while (received < total && received < (int)sizeof(buf) - 1) {
        int n = httpd_req_recv(req, buf + received, sizeof(buf) - 1 - received);
        if (n <= 0) break;
        received += n;
    }
    buf[received] = 0;
    ESP_LOGI(TAG, "/mode POST received %d bytes: '%s'", received, buf);

    if (received <= 0) {
        httpd_resp_sendstr(req, "ERR: no data");
        return ESP_OK;
    }
    if (strstr(buf, "auto")) {
        auto_mode = true;
        ESP_LOGI(TAG, "模式切换: 自动 (auto_mode=%d)", auto_mode);
    } else if (strstr(buf, "manual")) {
        auto_mode = false;
        ESP_LOGI(TAG, "模式切换: 手动 (auto_mode=%d)", auto_mode);
    } else {
        ESP_LOGW(TAG, "/mode 未知内容: '%s'", buf);
    }
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

/* /cmd 接口 */
static esp_err_t cmd_get_handler(httpd_req_t *req)
{
    char buf[16] = {0};
    httpd_req_get_url_query_str(req, buf, sizeof(buf));
    char cmd = 0;
    for (int i = 0; buf[i]; i++) {
        if (buf[i] == '=' && buf[i+1]) { cmd = buf[i+1]; break; }
    }
    if (!cmd) { httpd_resp_sendstr(req, "ERR: no cmd"); return ESP_OK; }
    /* send_cmd 内部已乐观更新状态并始终返回 OK */
    send_cmd(cmd);
    ESP_LOGI(TAG, "HTTP GET /cmd?c=%c", cmd);
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 10;
    httpd_handle_t s = NULL;
    if (httpd_start(&s, &config) != ESP_OK) return NULL;
    httpd_uri_t root   = {.uri="/",        .method=HTTP_GET,  .handler=root_get_handler};
    httpd_uri_t info   = {.uri="/info",    .method=HTTP_GET,  .handler=info_get_handler};
    httpd_uri_t cfg    = {.uri="/config",  .method=HTTP_POST, .handler=config_post_handler};
    httpd_uri_t cmd    = {.uri="/cmd",     .method=HTTP_GET,  .handler=cmd_get_handler};
    httpd_uri_t status = {.uri="/status",  .method=HTTP_GET,  .handler=status_get_handler};
    httpd_uri_t scan   = {.uri="/scan",    .method=HTTP_GET,  .handler=scan_get_handler};
    httpd_uri_t dist   = {.uri="/distance",.method=HTTP_GET,  .handler=distance_get_handler};
    httpd_uri_t thresh = {.uri="/threshold",.method=HTTP_GET, .handler=threshold_get_handler};
    httpd_uri_t mode   = {.uri="/mode",    .method=HTTP_POST, .handler=mode_post_handler};
    httpd_register_uri_handler(s, &root);
    httpd_register_uri_handler(s, &info);
    httpd_register_uri_handler(s, &cfg);
    httpd_register_uri_handler(s, &cmd);
    httpd_register_uri_handler(s, &status);
    httpd_register_uri_handler(s, &scan);
    httpd_register_uri_handler(s, &thresh);
    httpd_register_uri_handler(s, &dist);
    httpd_register_uri_handler(s, &mode);
    return s;
}

/* ========== WiFi 事件 ========== */
static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT) {
        if (id == WIFI_EVENT_STA_START) {
            esp_wifi_connect();
        } else if (id == WIFI_EVENT_STA_DISCONNECTED) {
            sta_connected = false;
            strcpy(sta_ip, "0.0.0.0");
            xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
            ESP_LOGW(TAG, "STA disconnected, retrying...");
            esp_wifi_connect();
        }
    } else if (base == IP_EVENT) {
        if (id == IP_EVENT_STA_GOT_IP) {
            ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
            /* v6.0.2: 不用 IP2STR 宏（与 BSD struct in_addr 冲突），用 ip4addr_ntoa_r */
            ip4addr_ntoa_r((const ip4_addr_t *)&e->ip_info.ip, sta_ip, sizeof(sta_ip));
            sta_connected = true;
            xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
            ESP_LOGI(TAG, "STA got IP: %s", sta_ip);
            ESP_LOGI(TAG, "========================================");
            ESP_LOGI(TAG, "S3 就绪! IP=%s", sta_ip);
            ESP_LOGI(TAG, "控制: http://%s/cmd?c=1|0|T", sta_ip);
            ESP_LOGI(TAG, "========================================");
        }
    }
}

/* ========== 主函数 ========== */
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* 初始化 HC-SR04 超声波传感器 GPIO */
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << TRIG_GPIO);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level(TRIG_GPIO, 0);

    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << ECHO_GPIO);
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(&io_conf);
    ESP_LOGI(TAG, "HC-SR04 GPIO 初始化: Trig=%d, Echo=%d", TRIG_GPIO, ECHO_GPIO);

    wifi_event_group = xEventGroupCreate();

    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();

    ESP_ERROR_CHECK(esp_netif_dhcps_stop(ap_netif));
    esp_netif_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip, 192,168,4,1);
    IP4_ADDR(&ip_info.gw, 192,168,4,1);
    IP4_ADDR(&ip_info.netmask, 255,255,255,0);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif, &ip_info));
    ESP_ERROR_CHECK(esp_netif_dhcps_start(ap_netif));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));

    wifi_config_t ap_config = {
        .ap = {
            .ssid = "ESP32_Setup",
            .password = "",
            .max_connection = 4,
            .authmode = WIFI_AUTH_OPEN,
            .channel = 0,  /* 自动跟随 STA 信道 */
            .ssid_hidden = 0,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));

    char ssid[33] = {0}, pass[65] = {0};
    bool has_config = (nvs_load_wifi(ssid, sizeof(ssid), pass, sizeof(pass)) == ESP_OK && ssid[0]);

    if (has_config) {
        is_config_mode = false;
        strncpy(sta_ssid, ssid, 32);

        wifi_config_t sta_config = {};
        strncpy((char *)sta_config.sta.ssid, ssid, sizeof(sta_config.sta.ssid) - 1);
        strncpy((char *)sta_config.sta.password, pass, sizeof(sta_config.sta.password) - 1);
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
        ESP_LOGI(TAG, "Loaded WiFi config: %s", ssid);

        ESP_ERROR_CHECK(esp_wifi_start());
        ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(80));  /* 20dBm 最大功率 */
        ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

        ESP_LOGI(TAG, "Waiting for WiFi connection (15s timeout)...");
        EventBits_t bits = xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT,
                                                pdFALSE, pdTRUE, 15000 / portTICK_PERIOD_MS);
        if (bits & WIFI_CONNECTED_BIT) {
            ESP_LOGI(TAG, "WiFi connected!");
        } else {
            ESP_LOGW(TAG, "WiFi timeout, entering config mode");
            is_config_mode = true;
            xTaskCreate(dns_server_task, "dns_server", 4096, NULL, 5, NULL);
        }
    } else {
        is_config_mode = true;
        ESP_ERROR_CHECK(esp_wifi_start());
        ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(80));  /* 20dBm 最大功率 */
        ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
        ESP_LOGI(TAG, "No WiFi config, config mode");
        xTaskCreate(dns_server_task, "dns_server", 4096, NULL, 5, NULL);
    }

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    /* 添加 C3 peer（硬编码 MAC，运行时动态更新） */
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, c3_mac, 6);
    peer.channel = 0;
    peer.ifidx = WIFI_IF_AP;
    peer.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    /* 添加广播 peer（后备：MAC 不匹配时用广播发送） */
    memcpy(peer.peer_addr, broadcast_mac, 6);
    esp_now_add_peer(&peer);  /* 广播 peer 可能返回错误, 忽略 */

    start_webserver();

    /* 启动 UDP 设备发现服务 (正常模式下在STA网络上发现) */
    xTaskCreate(udp_discovery_task, "udp_disc", 4096, NULL, 4, NULL);

    /* 启动 UDP 监听: 接收 C3 广播的 IP */
    xTaskCreate(udp_listen_task, "udp_listen", 4096, NULL, 4, NULL);

    /* 启动超声波采集 + 自动控制任务 */
    xTaskCreate(ultrasonic_task, "ultrasonic", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "S3 控制端就绪");
    if (is_config_mode) {
        ESP_LOGI(TAG, "配网模式: AP 'ESP32_Setup' (开放, 192.168.4.1)");
        ESP_LOGI(TAG, "手机连ESP32_Setup → 浏览器自动跳转配网页");
    } else {
        ESP_LOGI(TAG, "正常模式: STA IP=%s", sta_ip);
        ESP_LOGI(TAG, "控制: http://%s/cmd?c=1|0|T", sta_ip);
    }
    ESP_LOGI(TAG, "========================================");
}