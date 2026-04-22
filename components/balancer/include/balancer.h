// Self-balancing controller.
//
// Single 200 Hz task pinned to core 1:
//   - update IMU, snapshot accel + gyro (one coherent sample)
//   - complementary filter -> tilt angle (deg)
//   - cascade PID: angle PID + speed PID -> clipped PWM
//   - safety: arming state machine, tilt cutoff, I²C-fault disarm
//
// All tunables are exposed via `idf.py menuconfig` under
// "Bala2 balancer". Axis mapping and motor polarity are config
// options so hardware quirks don't require code edits.

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BALANCER_BOOT,        // Not started
    BALANCER_DISARMED,    // Idle (motors at 0, waiting for operator)
    BALANCER_ARMING,      // Waiting for upright + still
    BALANCER_ARMED,       // Running PID, motors live
    BALANCER_CRASHED,     // Tilt cutoff tripped; manual re-arm required
    BALANCER_FAULT,       // Comms / sensor fault; manual re-arm required
} balancer_state_t;

typedef struct {
    balancer_state_t state;
    float angle_deg;          // Fused tilt angle (0 = upright)
    float gyro_dps;            // Tilt-axis gyro rate
    float accel_mag_g;         // |accel| in g (sanity)
    int16_t pwm_left;
    int16_t pwm_right;
    int32_t enc_left;
    int32_t enc_right;
    float loop_dt_ms;          // Most recent measured dt
    uint32_t overrun_count;
    uint32_t i2c_err_count;
    uint32_t loops;
} balancer_status_t;

// Start the balancer task. Call after M5.begin() and bala2_base_init().
// Returns ESP_OK once the task is launched.
esp_err_t balancer_start(void);

// Operator controls. Safe to call from any task.
// arm()  : if DISARMED, transitions to ARMING (then ARMED once stable).
//          No-op otherwise.
// disarm(): unconditionally DISARMED, motors to 0.
void balancer_arm(void);
void balancer_disarm(void);
bool balancer_is_armed(void);

// Telemetry snapshot (copied out; thread-safe).
void balancer_get_status(balancer_status_t *out);

#ifdef __cplusplus
}
#endif
