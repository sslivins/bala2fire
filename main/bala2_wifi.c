#include "bala2_wifi.h"

#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

static const char *TAG = "bala2_wifi";

static bala2_wifi_status_t s_status = {
    .state = BALA2_WIFI_NOT_CONFIGURED,
    .ip_str = "",
};

static portMUX_TYPE s_status_lock = portMUX_INITIALIZER_UNLOCKED;

static void set_state(bala2_wifi_state_t state, const char *ip)
{
    taskENTER_CRITICAL(&s_status_lock);
    s_status.state = state;
    if (ip) {
        strncpy(s_status.ip_str, ip, sizeof(s_status.ip_str) - 1);
        s_status.ip_str[sizeof(s_status.ip_str) - 1] = '\0';
    } else {
        s_status.ip_str[0] = '\0';
    }
    taskEXIT_CRITICAL(&s_status_lock);
}

void bala2_wifi_get_status(bala2_wifi_status_t *out)
{
    if (!out) return;
    taskENTER_CRITICAL(&s_status_lock);
    *out = s_status;
    taskEXIT_CRITICAL(&s_status_lock);
}

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        // Only mark as failed once the startup timeout elapses (handled
        // in the start task). On transient disconnects, try to rejoin.
        ESP_LOGW(TAG, "disconnected, retrying...");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *evt = (ip_event_got_ip_t *)data;
        char ip[16];
        snprintf(ip, sizeof(ip), IPSTR, IP2STR(&evt->ip_info.ip));
        ESP_LOGI(TAG, "got IP: %s", ip);
        set_state(BALA2_WIFI_CONNECTED, ip);
    }
}

static void wifi_start_task(void *arg)
{
    const char *ssid = CONFIG_BALA2_WIFI_SSID;
    const char *pass = CONFIG_BALA2_WIFI_PASSWORD;

    if (ssid[0] == '\0') {
        ESP_LOGI(TAG, "SSID is empty; skipping Wi-Fi bring-up");
        set_state(BALA2_WIFI_NOT_CONFIGURED, NULL);
        vTaskDelete(NULL);
        return;
    }

    set_state(BALA2_WIFI_CONNECTING, NULL);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &on_wifi_event, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &on_wifi_event, NULL, NULL));

    wifi_config_t wifi_cfg = { 0 };
    strncpy((char *)wifi_cfg.sta.ssid, ssid, sizeof(wifi_cfg.sta.ssid) - 1);
    strncpy((char *)wifi_cfg.sta.password, pass, sizeof(wifi_cfg.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "connecting to \"%s\"...", ssid);

    const int64_t deadline_us =
        esp_timer_get_time() + (int64_t)CONFIG_BALA2_WIFI_CONNECT_TIMEOUT_MS * 1000;

    while (esp_timer_get_time() < deadline_us) {
        bala2_wifi_status_t cur;
        bala2_wifi_get_status(&cur);
        if (cur.state == BALA2_WIFI_CONNECTED) {
            vTaskDelete(NULL);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    bala2_wifi_status_t cur;
    bala2_wifi_get_status(&cur);
    if (cur.state != BALA2_WIFI_CONNECTED) {
        ESP_LOGW(TAG, "connect timeout");
        set_state(BALA2_WIFI_FAILED, NULL);
    }
    vTaskDelete(NULL);
}

void bala2_wifi_start(void)
{
    xTaskCreatePinnedToCore(wifi_start_task, "wifi_start", 4096, NULL, 5, NULL, 0);
}
