#include <M5Unified.h>
#include "esp_mac.h"
#include "bala2_wifi.h"

extern "C" void app_main(void)
{
    auto cfg = M5.config();
    M5.begin(cfg);

    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);

    M5.Display.setTextSize(2);
    M5.Display.setCursor(10, 10);
    M5.Display.print("Bala2 Fire");
    M5.Display.setCursor(10, 40);
    M5.Display.print("Hello, world!");
    M5.Display.setCursor(10, 70);
    M5.Display.printf("WiFi MAC:");
    M5.Display.setCursor(10, 95);
    M5.Display.printf("%02X:%02X:%02X:%02X:%02X:%02X",
                      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    bala2_wifi_start();

    bala2_wifi_state_t last_state = (bala2_wifi_state_t)-1;
    char last_ip[16] = "";
    int count = 0;

    while (true) {
        M5.update();

        // Redraw Wi-Fi line when status changes.
        bala2_wifi_status_t w;
        bala2_wifi_get_status(&w);
        if (w.state != last_state || strcmp(w.ip_str, last_ip) != 0) {
            last_state = w.state;
            strncpy(last_ip, w.ip_str, sizeof(last_ip));

            M5.Display.fillRect(10, 125, 310, 25, TFT_BLACK);
            M5.Display.setCursor(10, 125);
            switch (w.state) {
                case BALA2_WIFI_NOT_CONFIGURED:
                    M5.Display.print("IP: not configured");
                    break;
                case BALA2_WIFI_CONNECTING:
                    M5.Display.print("IP: connecting...");
                    break;
                case BALA2_WIFI_CONNECTED:
                    M5.Display.printf("IP: %s", w.ip_str);
                    break;
                case BALA2_WIFI_FAILED:
                    M5.Display.print("IP: connect failed");
                    break;
            }
        }

        if (M5.BtnA.wasPressed()) {
            M5.Display.fillRect(10, 160, 300, 30, TFT_BLACK);
            M5.Display.setCursor(10, 160);
            M5.Display.printf("A pressed #%d", ++count);
        }
        if (M5.BtnB.wasPressed()) {
            M5.Speaker.tone(1000, 100);
        }
        if (M5.BtnC.wasPressed()) {
            float ax, ay, az;
            M5.Imu.getAccelData(&ax, &ay, &az);
            M5.Display.fillRect(10, 200, 300, 30, TFT_BLACK);
            M5.Display.setCursor(10, 200);
            M5.Display.printf("ax=%.2f ay=%.2f", ax, ay);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
