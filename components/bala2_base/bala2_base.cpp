#include "bala2_base.h"

#include <M5Unified.h>
#include "esp_log.h"

static const char *TAG = "bala2_base";

static constexpr uint8_t I2C_ADDR           = 0x3A;
static constexpr uint8_t REG_MOTOR_SPEED    = 0x00;
static constexpr uint8_t REG_ENCODERS       = 0x10;
static constexpr uint8_t REG_SERVO_ANGLE    = 0x20;
static constexpr uint8_t REG_SERVO_PULSE    = 0x30;
static constexpr uint32_t I2C_FREQ_HZ       = 400000;

static SemaphoreHandle_t s_lock = nullptr;

extern "C" esp_err_t bala2_base_init(void)
{
    if (s_lock) return ESP_OK;
    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) return ESP_ERR_NO_MEM;
    ESP_LOGI(TAG, "ready (addr=0x%02X, %lu Hz via M5.In_I2C)",
             I2C_ADDR, (unsigned long)I2C_FREQ_HZ);
    return ESP_OK;
}

extern "C" void bala2_base_lock_take(void) {
    if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY);
}
extern "C" void bala2_base_lock_give(void) {
    if (s_lock) xSemaphoreGive(s_lock);
}

static esp_err_t write_reg(uint8_t reg, const uint8_t *data, size_t len)
{
    if (!s_lock) return ESP_ERR_INVALID_STATE;
    bala2_base_lock_take();
    bool ok = M5.In_I2C.writeRegister(I2C_ADDR, reg, data, len, I2C_FREQ_HZ);
    bala2_base_lock_give();
    return ok ? ESP_OK : ESP_FAIL;
}

static esp_err_t read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    if (!s_lock) return ESP_ERR_INVALID_STATE;
    bala2_base_lock_take();
    bool ok = M5.In_I2C.readRegister(I2C_ADDR, reg, data, len, I2C_FREQ_HZ);
    bala2_base_lock_give();
    return ok ? ESP_OK : ESP_FAIL;
}

extern "C" esp_err_t bala2_base_set_speed(int16_t left, int16_t right)
{
    if (left  >  1023) left  =  1023;
    if (left  < -1023) left  = -1023;
    if (right >  1023) right =  1023;
    if (right < -1023) right = -1023;

    uint8_t data[4] = {
        (uint8_t)((uint16_t)left  >> 8), (uint8_t)((uint16_t)left  & 0xFF),
        (uint8_t)((uint16_t)right >> 8), (uint8_t)((uint16_t)right & 0xFF),
    };
    return write_reg(REG_MOTOR_SPEED, data, sizeof(data));
}

extern "C" esp_err_t bala2_base_get_encoders(int32_t *left, int32_t *right)
{
    uint8_t data[8];
    esp_err_t err = read_reg(REG_ENCODERS, data, sizeof(data));
    if (err != ESP_OK) return err;

    if (left) {
        *left = (int32_t)(((uint32_t)data[0] << 24) |
                          ((uint32_t)data[1] << 16) |
                          ((uint32_t)data[2] << 8)  |
                           (uint32_t)data[3]);
    }
    if (right) {
        *right = (int32_t)(((uint32_t)data[4] << 24) |
                           ((uint32_t)data[5] << 16) |
                           ((uint32_t)data[6] << 8)  |
                            (uint32_t)data[7]);
    }
    return ESP_OK;
}

extern "C" esp_err_t bala2_base_set_encoders(int32_t left, int32_t right)
{
    uint8_t data[8] = {
        (uint8_t)((uint32_t)left  >> 24), (uint8_t)((uint32_t)left  >> 16),
        (uint8_t)((uint32_t)left  >> 8),  (uint8_t)((uint32_t)left  & 0xFF),
        (uint8_t)((uint32_t)right >> 24), (uint8_t)((uint32_t)right >> 16),
        (uint8_t)((uint32_t)right >> 8),  (uint8_t)((uint32_t)right & 0xFF),
    };
    return write_reg(REG_ENCODERS, data, sizeof(data));
}

extern "C" esp_err_t bala2_base_set_servo_angle(uint8_t pos, uint8_t angle)
{
    if (pos > 7) return ESP_ERR_INVALID_ARG;
    if (angle > 180) angle = 180;
    return write_reg((uint8_t)(REG_SERVO_ANGLE + pos), &angle, 1);
}

extern "C" esp_err_t bala2_base_set_servo_pulse(uint8_t pos, uint16_t width)
{
    if (pos > 7) return ESP_ERR_INVALID_ARG;
    if (width < 500)  width = 500;
    if (width > 2500) width = 2500;
    uint8_t data[2] = { (uint8_t)(width >> 8), (uint8_t)(width & 0xFF) };
    return write_reg((uint8_t)(REG_SERVO_PULSE + (pos * 2)), data, sizeof(data));
}
