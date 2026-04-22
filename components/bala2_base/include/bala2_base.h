// Driver for the Bala2 Fire wheel base.
//
// The wheel base has an STM32 slave on I²C addr 0x3A (shared with the
// MPU6886 on the Fire's internal bus, GPIO21/22). It handles low-level
// motor PWM + wheel encoders.
//
// Register map (from m5stack/M5Bala2 reference firmware):
//   0x00 W 4B   int16 BE left PWM, int16 BE right PWM  (-1023..+1023)
//   0x10 R/W 8B int32 BE left encoder, int32 BE right encoder
//   0x20+pos   W 1B  servo angle 0..180   (pos = 0..7)
//   0x30+pos*2 W 2B  servo pulse width µs (500..2500)
//
// All public functions are thread-safe: they grab an internal mutex.
// Callers that also touch the same I²C bus directly (e.g. via M5.Imu)
// MUST wrap their accesses with bala2_base_lock_take / _give so we
// don't collide with them.

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Prepare the shared bus mutex. Call once from app_main after
// M5.begin(). Idempotent.
esp_err_t bala2_base_init(void);

// Take / give the shared bus mutex. Anything that also uses the Fire's
// internal I²C bus from outside this component should wrap its
// accesses here.
void bala2_base_lock_take(void);
void bala2_base_lock_give(void);

// Send signed PWM (-1023..+1023) to both wheels. The STM32 latches
// the value until overwritten, so callers MUST keep calling this at
// a steady rate (or explicitly send 0 on disarm). Returns ESP_OK on
// success; any failure should cause the caller to disarm.
esp_err_t bala2_base_set_speed(int16_t left, int16_t right);

// Read raw encoder counts from the base. Counts are signed int32 that
// wrap mod 2^32; compute deltas with UNSIGNED subtraction.
esp_err_t bala2_base_get_encoders(int32_t *left, int32_t *right);

// Overwrite the base's encoder accumulator (both wheels).
esp_err_t bala2_base_set_encoders(int32_t left, int32_t right);

// Convenience: zero both encoders.
static inline esp_err_t bala2_base_clear_encoders(void) {
    return bala2_base_set_encoders(0, 0);
}

// Servo helpers. pos = 0..7, angle = 0..180°, pulse = 500..2500 µs.
esp_err_t bala2_base_set_servo_angle(uint8_t pos, uint8_t angle);
esp_err_t bala2_base_set_servo_pulse(uint8_t pos, uint16_t width);

#ifdef __cplusplus
}
#endif
