#include <M5Unified.h>

extern "C" void app_main(void)
{
    auto cfg = M5.config();
    M5.begin(cfg);

    M5.Display.setTextSize(2);
    M5.Display.setCursor(10, 10);
    M5.Display.print("Bala2 Fire");
    M5.Display.setCursor(10, 40);
    M5.Display.print("Hello, world!");

    int count = 0;
    while (true) {
        M5.update();

        if (M5.BtnA.wasPressed()) {
            M5.Display.fillRect(10, 80, 300, 30, TFT_BLACK);
            M5.Display.setCursor(10, 80);
            M5.Display.printf("A pressed #%d", ++count);
        }
        if (M5.BtnB.wasPressed()) {
            M5.Speaker.tone(1000, 100);
        }
        if (M5.BtnC.wasPressed()) {
            float ax, ay, az;
            M5.Imu.getAccelData(&ax, &ay, &az);
            M5.Display.fillRect(10, 120, 300, 30, TFT_BLACK);
            M5.Display.setCursor(10, 120);
            M5.Display.printf("ax=%.2f ay=%.2f", ax, ay);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
