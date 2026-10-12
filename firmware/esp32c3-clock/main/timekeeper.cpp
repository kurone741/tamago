#include "timekeeper.h"

#include <atomic>
#include <cstdlib>
#include <cstring>
#include <sys/time.h>
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

namespace tk {
static const char* TAG = "tk";
static std::atomic<bool> s_valid{false};
static std::atomic<bool> s_conn{false};
static bool s_sntp_started = false;
static esp_timer_handle_t s_retry = nullptr;

static void retry_cb(void*) { esp_wifi_connect(); }
static void sync_cb(struct timeval*) { s_valid = true; ESP_LOGI(TAG, "SNTP synced"); }

static void on_event(void*, esp_event_base_t base, int32_t id, void*) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_conn = false;
        esp_timer_start_once(s_retry, 5 * 1000 * 1000);   // retry every 5 s
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        s_conn = true;
        if (!s_sntp_started) {
            s_sntp_started = true;
            esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
            esp_sntp_setservername(0, CONFIG_CLOCK_NTP_SERVER);
            esp_sntp_set_time_sync_notification_cb(sync_cb);
            esp_sntp_init();
        }
    }
}

void init() {
    setenv("TZ", CONFIG_CLOCK_TZ, 1);
    tzset();

    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    if (strlen(CONFIG_CLOCK_WIFI_SSID) == 0) {
        ESP_LOGW(TAG, "No Wi-Fi SSID configured; manual time only");
        return;
    }

    const esp_timer_create_args_t ta = {.callback = retry_cb, .name = "wifi_retry"};
    esp_timer_create(&ta, &s_retry);

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_event, nullptr);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_event, nullptr);

    wifi_config_t wc = {};
    strlcpy(reinterpret_cast<char*>(wc.sta.ssid), CONFIG_CLOCK_WIFI_SSID, sizeof(wc.sta.ssid));
    strlcpy(reinterpret_cast<char*>(wc.sta.password), CONFIG_CLOCK_WIFI_PASSWORD, sizeof(wc.sta.password));
    wc.sta.threshold.authmode = strlen(CONFIG_CLOCK_WIFI_PASSWORD) ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wc);
    esp_wifi_start();
}

bool time_valid() { return s_valid; }
bool wifi_connected() { return s_conn; }

void set_manual(const struct tm& local) {
    struct tm t = local;
    t.tm_isdst = -1;
    struct timeval tv = {.tv_sec = mktime(&t), .tv_usec = 0};
    settimeofday(&tv, nullptr);
    s_valid = true;
}
}  // namespace tk
