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
    BALANCER_EXTERNAL,    // Host-controlled raw PWM (on-device PID bypassed,
                          // tilt-cutoff + I²C-fault safety still active)
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

// Drive command. Both axes are normalized to [-1.0, +1.0] and clamped.
//   linear  : forward (+) / backward (-) bias on the tilt setpoint.
//             A drive command lasts ~500 ms; resend at >2 Hz to keep
//             moving. Loss of caller -> robot decelerates to balance.
//   angular : right (+) / left (-) yaw, adds differential PWM bias.
// Has effect only while ARMED.
void balancer_set_drive(float linear, float angular);

// Telemetry snapshot (copied out; thread-safe).
void balancer_get_status(balancer_status_t *out);

// ---- Host-side control ("external mode") ----
//
// Lets a host script run the PID loop. Workflow:
//   1. balancer_set_external(true)   // requires DISARMED/CRASHED/FAULT; -> EXTERNAL
//   2. balancer_get_imu(...) at >50 Hz, compute PWM, balancer_set_pwm(l, r)
//   3. balancer_set_external(false)  // -> DISARMED
//
// While EXTERNAL: on-device PID is OFF, motors track caller's set_pwm()
// values, but the device still:
//   * reads IMU + encoders every loop tick (visible via _get_imu / _get_status)
//   * trips tilt-cutoff -> CRASHED if |angle| > limit
//   * trips fault if I²C fails
//   * watchdogs set_pwm: if no update for ~200 ms, motors -> 0
//   * accepts balancer_disarm() at any time (-> DISARMED)

typedef struct {
    float angle_deg;          // fused tilt (0 = upright)
    float gyro_dps;            // tilt-axis rate, bias-corrected
    float accel_x, accel_y, accel_z;   // raw accel, g
    float gyro_x, gyro_y, gyro_z;      // raw gyro, dps (pre-bias)
    float gyro_bias_dps;        // current learned bias on tilt axis
    float loop_dt_ms;
    uint32_t loops;
    balancer_state_t state;
} balancer_imu_t;

void balancer_get_imu(balancer_imu_t *out);

// Enter / leave EXTERNAL mode.
// enable=true requires current state in {DISARMED, CRASHED, FAULT}.
// enable=false unconditionally returns to DISARMED.
// Returns ESP_OK or ESP_ERR_INVALID_STATE.
esp_err_t balancer_set_external(bool enable);

// Set raw PWM. Only effective in EXTERNAL state. Each call refreshes a
// ~200 ms watchdog: stop calling and motors decay to 0. Returns
// ESP_ERR_INVALID_STATE outside EXTERNAL.
esp_err_t balancer_set_pwm(int16_t left, int16_t right);

#ifdef __cplusplus
}
#endif
