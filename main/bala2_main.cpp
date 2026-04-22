#include <M5Unified.h>
#include <cstring>
#include <cstdio>

#include "esp_log.h"
#include "esp_mac.h"

#include "bala2_wifi.h"
#include "bala2_base.h"
#include "balancer.h"

static const char *TAG = "bala2";

static const char *state_str(balancer_state_t s) {
    switch (s) {
        case BALANCER_BOOT:     return "BOOT";
        case BALANCER_DISARMED: return "OFF";
        case BALANCER_ARMING:   return "ARMING";
        case BALANCER_ARMED:    return "ON";
        case BALANCER_CRASHED:  return "CRASH";
        case BALANCER_FAULT:    return "FAULT";
    }
    return "?";
}

static uint32_t state_color(balancer_state_t s) {
    switch (s) {
        case BALANCER_ARMED:    return TFT_GREEN;
        case BALANCER_ARMING:   return TFT_YELLOW;
        case BALANCER_CRASHED:  return TFT_RED;
        case BALANCER_FAULT:    return TFT_ORANGE;
        case BALANCER_DISARMED:
        default:                return TFT_DARKGREY;
    }
}

// The M5Stack Fire's three physical buttons (A/B/C) sit below the
// 320x240 TFT, roughly centered under x ≈ 60 / 160 / 260.
// We draw a status pill just above Button A so the operator can tell
// at a glance whether the self-balancing loop is engaged.
static void draw_bal_indicator(balancer_state_t s) {
    const int x = 10, y = 205, w = 100, h = 30;
    uint32_t col = state_color(s);
    M5.Display.fillRoundRect(x, y, w, h, 6, col);
    M5.Display.drawRoundRect(x, y, w, h, 6, TFT_WHITE);
    M5.Display.setTextColor(TFT_BLACK, col);
    M5.Display.setTextSize(2);
    const char *label = state_str(s);
    int tw = (int)strlen(label) * 12; // 6px glyph × size 2
    int tx = x + (w - tw) / 2;
    if (tx < x + 2) tx = x + 2;
    M5.Display.setCursor(tx, y + 8);
    M5.Display.printf("%s", label);
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
}

extern "C" void app_main(void)
{
    auto cfg = M5.config();
    M5.begin(cfg);
    M5.Display.setRotation(1);
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);

    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);

    M5.Display.setTextSize(2);
    M5.Display.setCursor(10, 6);
    M5.Display.print("Bala2 Fire");
    M5.Display.setCursor(10, 30);
    M5.Display.printf("MAC %02X:%02X:%02X:%02X:%02X:%02X",
                      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    ESP_ERROR_CHECK(bala2_base_init());
    ESP_ERROR_CHECK(balancer_start());
    bala2_wifi_start();

    ESP_LOGI(TAG, "app_main setup complete");

    bala2_wifi_state_t last_wstate = (bala2_wifi_state_t)-1;
    char last_ip[16] = "";
    balancer_state_t last_bstate = (balancer_state_t)-1;

    int redraw_divider = 0;

    while (true) {
        M5.update();

        bala2_wifi_status_t w;
        bala2_wifi_get_status(&w);
        if (w.state != last_wstate || strcmp(w.ip_str, last_ip) != 0) {
            last_wstate = w.state;
            strncpy(last_ip, w.ip_str, sizeof(last_ip));

            M5.Display.fillRect(10, 54, 310, 24, TFT_BLACK);
            M5.Display.setCursor(10, 54);
            switch (w.state) {
                case BALA2_WIFI_NOT_CONFIGURED: M5.Display.print("IP: not configured"); break;
                case BALA2_WIFI_CONNECTING:     M5.Display.print("IP: connecting..."); break;
                case BALA2_WIFI_CONNECTED:      M5.Display.printf("IP: %s", w.ip_str); break;
                case BALA2_WIFI_FAILED:         M5.Display.print("IP: connect failed"); break;
            }
        }

        balancer_status_t b;
        balancer_get_status(&b);

        if ((++redraw_divider % 10) == 0) {
            M5.Display.fillRect(10, 88, 310, 80, TFT_BLACK);
            M5.Display.setCursor(10, 88);
            M5.Display.printf("ang %+6.1f deg", b.angle_deg);
            M5.Display.setCursor(10, 112);
            M5.Display.printf("rate %+6.0f dps", b.gyro_dps);
            M5.Display.setCursor(10, 136);
            M5.Display.printf("pwm  %+5d / %+5d", b.pwm_left, b.pwm_right);
        }

        if (b.state != last_bstate) {
            last_bstate = b.state;
            draw_bal_indicator(b.state);
        }

        if (M5.BtnA.wasPressed()) {
            if (balancer_is_armed()) {
                balancer_disarm();
            } else {
                balancer_arm();
            }
        }
        if (M5.BtnB.wasPressed()) {
            M5.Speaker.tone(1000, 80);
        }
        if (M5.BtnC.wasPressed()) {
            // Emergency stop.
            balancer_disarm();
            M5.Speaker.tone(400, 250);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
