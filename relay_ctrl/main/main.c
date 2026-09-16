/**
 * ESP32-C3 继电器 ESP-NOW 接收端
 * 接收 ESP32-S3 的 ESP-NOW 指令，控制 GPIO4 继电器
 * 同时保留 HTTP 网页控制（本地调试用）
 *
 * STA+AP 模式：STA 连家庭 WiFi（与 S3 同信道），AP 保留供调试
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_http_server.h"
#include "esp_now.h"
#include "lwip/ip4_addr.h"
#include "lwip/sockets.h"
#include <netinet/in.h>

/* ===== 硬编码 WiFi 凭据（与 S3 相同的家庭路由器） ===== */
#define WIFI_SSID "ZTE-HfEcPY"
#define WIFI_PASS "liuch@1213577@"

static const char *TAG = "RELAY_RX";

#define RELAY_GPIO           4
static int relay_active_level = 1;
static int relay_on = 0;

/* WiFi 连接事件 */
#define WIFI_CONNECTED_BIT BIT0
static EventGroupHandle_t wifi_event_group;
static char sta_ip[16] = "0.0.0.0";

/* S3 主控 MAC (收到第一条指令时自动记录) */
static uint8_t s3_mac[6] = {0};
static bool s3_peer_added = false;
static volatile bool need_status_send = false;  /* 回传标志（回调中设置，主循环中发送） */

static void relay_set(int on)
{
    relay_on = on;
    int level = on ? relay_active_level : !relay_active_level;
    gpio_set_level(RELAY_GPIO, level);
    int rd = gpio_get_level(RELAY_GPIO);
    ESP_LOGI(TAG, "relay: %s set=%d read=%d (active=%s)",
             on ? "ON" : "OFF", level, rd, relay_active_level ? "HIGH" : "LOW");
}

/* 回传状态给 S3: [relay_on, relay_active_level] */
static void send_status_to_s3(void)
{
    uint8_t status[2] = { (uint8_t)relay_on, (uint8_t)relay_active_level };

    /* 优先用单播（学到的 S3 AP MAC），失败再用广播 */
    esp_err_t r = ESP_FAIL;
    if (s3_peer_added && s3_mac[0] != 0) {
        r = esp_now_send(s3_mac, status, 2);
        ESP_LOGI(TAG, "ESP-NOW send status (unicast) to S3: on=%d level=%d -> %s",
                 relay_on, relay_active_level, r==ESP_OK?"OK":"FAIL");
    }
    if (r != ESP_OK) {
        /* 单播失败，用广播 */
        uint8_t bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        r = esp_now_send(bcast, status, 2);
        ESP_LOGI(TAG, "ESP-NOW send status (broadcast) to S3: on=%d level=%d -> %s",
                 relay_on, relay_active_level, r==ESP_OK?"OK":"FAIL");
    }
}

static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (len < 1) return;

    /* 首次收到 S3 指令时, 记录其 MAC 并添加为 peer */
    if (!s3_peer_added && info && info->src_addr) {
        memcpy(s3_mac, info->src_addr, 6);
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, s3_mac, 6);
        peer.channel = 0;       /* 0 = 使用当前信道 */
        peer.ifidx = WIFI_IF_AP; /* 用 AP 接口（信道6，与 S3 AP 一致） */
        peer.encrypt = false;
        esp_err_t r = esp_now_add_peer(&peer);
        s3_peer_added = (r == ESP_OK);
        ESP_LOGI(TAG, "S3 peer added: %02X:%02X:%02X:%02X:%02X:%02X %s",
                 s3_mac[0],s3_mac[1],s3_mac[2],s3_mac[3],s3_mac[4],s3_mac[5],
                 s3_peer_added?"OK":"FAIL");
    }

    char cmd = (char)data[0];
    ESP_LOGI(TAG, "ESP-NOW recv: cmd=%c len=%d", cmd, len);
    switch (cmd) {
    case '1': case 'O': case 'o': relay_set(1); break;
    case '0': case 'F': case 'f': relay_set(0); break;
    case 'T': case 't': relay_active_level=!relay_active_level; relay_set(relay_on); break;
    case 'S': case 's': ESP_LOGI(TAG, "status: on=%d active=%d", relay_on, relay_active_level); break;
    default: ESP_LOGW(TAG, "unknown cmd: %c", cmd); break;
    }

    /* 设置回传标志（不在回调中直接调用 esp_now_send） */
    need_status_send = true;
}

static const char *HTML_PAGE =
"<!DOCTYPE html><html lang='zh-CN'><head><meta charset='UTF-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>C3 继电器</title>"
"<style>body{font-family:sans-serif;max-width:400px;margin:40px auto;padding:0 20px;text-align:center;background:#f0f2f5}"
".card{background:#fff;border-radius:16px;padding:30px 20px;box-shadow:0 4px 20px rgba(0,0,0,.1)}"
"h1{color:#333;margin:0 0 8px;font-size:22px}"
".status{font-size:18px;margin:20px 0;padding:16px;border-radius:12px;font-weight:bold}"
".on{background:#e8f5e9;color:#2e7d32}.off{background:#ffebee;color:#c62828}"
".level{font-size:14px;color:#666;margin-bottom:20px}"
"button{width:100%%;padding:16px;font-size:18px;border:none;border-radius:12px;margin:8px 0;cursor:pointer;font-weight:bold;color:#fff}"
".btn-on{background:#43a047}.btn-off{background:#e53935}.btn-level{background:#1e88e5}"
"</style></head><body><div class='card'>"
"<h1>ESP32-C3 继电器</h1>"
"<div id='status' class='status off'>加载中...</div>"
"<div class='level'>触发: <span id='lvl'>高电平</span></div>"
"<button class='btn-on' onclick=\"fetch('/on',{method:'POST'});refresh()\">吸合 ON</button>"
"<button class='btn-off' onclick=\"fetch('/off',{method:'POST'});refresh()\">断开 OFF</button>"
"<button class='btn-level' onclick=\"fetch('/level',{method:'POST'});refresh()\">切换电平</button>"
"</div><script>"
"function refresh(){fetch('/status').then(r=>r.json()).then(d=>{"
"var s=document.getElementById('status');s.textContent=d.on?'吸合':'断开';"
"s.className='status '+(d.on?'on':'off');"
"document.getElementById('lvl').textContent=d.level?'高电平':'低电平';})}"
"refresh();setInterval(refresh,1500);</script></body></html>";

static esp_err_t root_get_handler(httpd_req_t *req){httpd_resp_set_type(req,"text/html; charset=utf-8");httpd_resp_send(req,HTML_PAGE,HTTPD_RESP_USE_STRLEN);return ESP_OK;}
static esp_err_t on_post_handler(httpd_req_t *req){relay_set(1);httpd_resp_sendstr(req,"OK");return ESP_OK;}
static esp_err_t off_post_handler(httpd_req_t *req){relay_set(0);httpd_resp_sendstr(req,"OK");return ESP_OK;}
static esp_err_t level_post_handler(httpd_req_t *req){relay_active_level=!relay_active_level;relay_set(relay_on);httpd_resp_sendstr(req,"OK");return ESP_OK;}
static esp_err_t status_get_handler(httpd_req_t *req){char buf[64];snprintf(buf,sizeof(buf),"{\"on\":%d,\"level\":%d}",relay_on,relay_active_level);httpd_resp_set_type(req,"application/json");httpd_resp_sendstr(req,buf);return ESP_OK;}

/* /cmd?c=1 或 /cmd?c=0 GET 接口 (供 S3 HTTP 调用) */
static esp_err_t cmd_get_handler(httpd_req_t *req)
{
    char buf[32];
    if (httpd_req_get_url_query_len(req) > 0) {
        httpd_req_get_url_query_str(req, buf, sizeof(buf));
        char c_val[4] = {0};
        if (httpd_query_key_value(buf, "c", c_val, sizeof(c_val)) == ESP_OK) {
            ESP_LOGI(TAG, "HTTP /cmd c=%s", c_val);
            switch (c_val[0]) {
            case '1': case 'O': case 'o': relay_set(1); break;
            case '0': case 'F': case 'f': relay_set(0); break;
            case 'T': case 't': relay_active_level=!relay_active_level; relay_set(relay_on); break;
            default: break;
            }
            httpd_resp_sendstr(req, "OK");
            return ESP_OK;
        }
    }
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing param c");
    return ESP_OK;
}

static httpd_handle_t start_webserver(void)
{
    httpd_config_t config=HTTPD_DEFAULT_CONFIG();config.max_uri_handlers=8;
    httpd_handle_t server=NULL;
    if(httpd_start(&server,&config)!=ESP_OK)return NULL;
    httpd_uri_t root={.uri="/",.method=HTTP_GET,.handler=root_get_handler};
    httpd_uri_t on={.uri="/on",.method=HTTP_POST,.handler=on_post_handler};
    httpd_uri_t off={.uri="/off",.method=HTTP_POST,.handler=off_post_handler};
    httpd_uri_t level={.uri="/level",.method=HTTP_POST,.handler=level_post_handler};
    httpd_uri_t status={.uri="/status",.method=HTTP_GET,.handler=status_get_handler};
    httpd_uri_t cmd={.uri="/cmd",.method=HTTP_GET,.handler=cmd_get_handler};
    httpd_register_uri_handler(server,&root);httpd_register_uri_handler(server,&on);
    httpd_register_uri_handler(server,&off);httpd_register_uri_handler(server,&level);
    httpd_register_uri_handler(server,&status);httpd_register_uri_handler(server,&cmd);return server;
}

/* WiFi 事件回调 */
static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    static int retry_count = 0;
    if (base == WIFI_EVENT) {
        if (id == WIFI_EVENT_STA_DISCONNECTED) {
            retry_count++;
            ESP_LOGW(TAG, "STA disconnected (retry %d), waiting 3s...", retry_count);
            /* 延迟 3 秒再重连，避免路由器拒绝频繁重连 */
            vTaskDelay(pdMS_TO_TICKS(3000));
            esp_wifi_connect();
        }
    } else if (base == IP_EVENT) {
        if (id == IP_EVENT_STA_GOT_IP) {
            retry_count = 0;
            ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
            ip4addr_ntoa_r((const ip4_addr_t *)&e->ip_info.ip, sta_ip, sizeof(sta_ip));
            ESP_LOGI(TAG, "STA connected! IP=%s", sta_ip);
            xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        }
    }
}

/* UDP 广播任务: 每 5 秒广播自己的 IP，让 S3 发现 */
static void udp_broadcast_task(void *pv)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) { vTaskDelete(NULL); return; }

    int broadcast = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

    struct sockaddr_in dest;
    memset(&dest, 0, sizeof(dest));
    dest.sin_family = AF_INET;
    dest.sin_port = htons(3333);
    dest.sin_addr.s_addr = inet_addr("255.255.255.255");

    while (1) {
        if (strlen(sta_ip) > 0 && strcmp(sta_ip, "0.0.0.0") != 0) {
            char buf[64];
            snprintf(buf, sizeof(buf), "C3_IP:%s", sta_ip);
            sendto(sock, buf, strlen(buf), 0, (struct sockaddr *)&dest, sizeof(dest));
            ESP_LOGI(TAG, "UDP 广播: %s", buf);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    close(sock);
    vTaskDelete(NULL);
}

void app_main(void)
{
    gpio_config_t io_conf={.pin_bit_mask=(1ULL<<RELAY_GPIO),.mode=GPIO_MODE_OUTPUT,
        .pull_up_en=GPIO_PULLUP_DISABLE,.pull_down_en=GPIO_PULLDOWN_DISABLE,.intr_type=GPIO_INTR_DISABLE};
    gpio_config(&io_conf);relay_set(0);

    esp_err_t ret=nvs_flash_init();
    if(ret==ESP_ERR_NVS_NO_FREE_PAGES||ret==ESP_ERR_NVS_NEW_VERSION_FOUND){nvs_flash_erase();nvs_flash_init();}

    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* 同时创建 AP 和 STA 的 netif */
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();

    /* AP 固定 IP 192.168.4.1（保留供调试） */
    ESP_ERROR_CHECK(esp_netif_dhcps_stop(ap_netif));
    esp_netif_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip,192,168,4,1);IP4_ADDR(&ip_info.gw,192,168,4,1);IP4_ADDR(&ip_info.netmask,255,255,255,0);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif,&ip_info));
    ESP_ERROR_CHECK(esp_netif_dhcps_start(ap_netif));

    /* 注册事件 */
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_init_config_t cfg=WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* STA+AP 模式：STA 连家庭 WiFi（与 S3 同信道），AP 保留供调试 */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));

    /* STA 配置：家庭 WiFi（和 S3 完全相同的配置方式 + PMF 兼容） */
    wifi_config_t sta_config = {};
    strncpy((char *)sta_config.sta.ssid, WIFI_SSID, sizeof(sta_config.sta.ssid) - 1);
    strncpy((char *)sta_config.sta.password, WIFI_PASS, sizeof(sta_config.sta.password) - 1);
    sta_config.sta.pmf_cfg.capable = true;
    sta_config.sta.pmf_cfg.required = false;
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));

    /* AP 配置：RELAY-C3 调试热点（信道自动跟随 STA） */
    wifi_config_t ap_config={.ap={.ssid="RELAY-C3",.password="",.max_connection=4,.authmode=WIFI_AUTH_OPEN,.channel=0}};
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP,&ap_config));

    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(78)); /* 提高功率增加覆盖范围 */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_LOGI(TAG, "Connecting to WiFi: %s", WIFI_SSID);
    esp_wifi_connect();

    /* 等待 STA 连接（10秒超时，连不上也继续，AP 模式仍可用） */
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT,
                                            pdFALSE, pdTRUE, 10000 / portTICK_PERIOD_MS);
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi connected, IP=%s", sta_ip);
    } else {
        ESP_LOGW(TAG, "WiFi connect timeout, AP-only mode (调试用)");
    }

    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_read_mac(mac,ESP_MAC_WIFI_STA));
    ESP_LOGI(TAG,"========================================");
    ESP_LOGI(TAG,"C3 STA MAC: %02X:%02X:%02X:%02X:%02X:%02X",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    ESP_LOGI(TAG,"C3 AP MAC: %02X:%02X:%02X:%02X:%02X:%02X",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
    ESP_LOGI(TAG,"STA IP: %s", sta_ip);
    ESP_LOGI(TAG,"========================================");

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    /* 添加广播 peer（用于回传状态给 S3） */
    esp_now_peer_info_t bcast_peer = {};
    uint8_t bmac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    memcpy(bcast_peer.peer_addr, bmac, 6);
    bcast_peer.channel = 0;
    bcast_peer.ifidx = WIFI_IF_AP;
    bcast_peer.encrypt = false;
    esp_now_add_peer(&bcast_peer);

    uint8_t primary_chan;
    wifi_second_chan_t second_chan;
    esp_wifi_get_channel(&primary_chan, &second_chan);
    ESP_LOGI(TAG,"ESP-NOW 接收端已启动 (channel %d)", primary_chan);

    start_webserver();
    ESP_LOGI(TAG,"HTTP: http://192.168.4.1 (AP) 或 http://%s (STA)", sta_ip);
    ESP_LOGI(TAG,"=== C3 接收端就绪，等待 S3 控制 ===");

    /* 启动 UDP 广播任务: 告诉 S3 自己的 IP */
    xTaskCreate(udp_broadcast_task, "udp_bcast", 4096, NULL, 4, NULL);

    /* 主循环: 处理状态回传（不在回调中直接调用 esp_now_send） */
    while (1) {
        if (need_status_send) {
            need_status_send = false;
            send_status_to_s3();
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
