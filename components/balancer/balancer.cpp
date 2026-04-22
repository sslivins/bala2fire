#include "balancer.h"

#include <math.h>
#include <string.h>

#include <M5Unified.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#include "bala2_base.h"

static const char *TAG = "balancer";

// ---------- Config helpers ----------

static constexpr float RAD2DEG = 57.29577951308232f;

// Kconfig `bool` with `default n` isn't defined at all; normalize.
#ifndef CONFIG_BALANCER_ANGLE_DIR_POS
#define CONFIG_BALANCER_ANGLE_DIR_POS 0
#endif
#ifndef CONFIG_BALANCER_SPEED_DIR_POS
#define CONFIG_BALANCER_SPEED_DIR_POS 0
#endif
#ifndef CONFIG_BALANCER_TILT_ACCEL_INVERT
#define CONFIG_BALANCER_TILT_ACCEL_INVERT 0
#endif
#ifndef CONFIG_BALANCER_TILT_GYRO_INVERT
#define CONFIG_BALANCER_TILT_GYRO_INVERT 0
#endif
#ifndef CONFIG_BALANCER_MOTOR_INVERT_LEFT
#define CONFIG_BALANCER_MOTOR_INVERT_LEFT 0
#endif
#ifndef CONFIG_BALANCER_MOTOR_INVERT_RIGHT
#define CONFIG_BALANCER_MOTOR_INVERT_RIGHT 0
#endif

static inline float kp_angle()  { return CONFIG_BALANCER_ANGLE_KP_MILLI / 1000.0f; }
static inline float ki_angle()  { return CONFIG_BALANCER_ANGLE_KI_MILLI / 1000.0f; }
static inline float kd_angle()  { return CONFIG_BALANCER_ANGLE_KD_MILLI / 1000.0f; }
static inline float kp_speed()  { return CONFIG_BALANCER_SPEED_KP_MILLI / 1000.0f; }
static inline float ki_speed()  { return CONFIG_BALANCER_SPEED_KI_MILLI / 1000.0f; }
static inline float kd_speed()  { return CONFIG_BALANCER_SPEED_KD_MILLI / 1000.0f; }
static inline float setpoint_deg() { return CONFIG_BALANCER_SETPOINT_MDEG / 1000.0f; }

static inline float angle_dir() { return CONFIG_BALANCER_ANGLE_DIR_POS ? +1.0f : -1.0f; }
static inline float speed_dir() { return CONFIG_BALANCER_SPEED_DIR_POS ? +1.0f : -1.0f; }

// Pick an axis from an (x, y, z) triplet based on Kconfig.
static inline float pick_accel_axis(float x, float y, float z) {
#if CONFIG_BALANCER_TILT_ACCEL_AXIS_X
    float v = x;
#elif CONFIG_BALANCER_TILT_ACCEL_AXIS_Z
    float v = z;
#else
    float v = y;
#endif
#if CONFIG_BALANCER_TILT_ACCEL_INVERT
    v = -v;
#endif
    return v;
}
// The "other two" axes (for atan2 denominator magnitude).
static inline void pick_other_accel_axes(float x, float y, float z,
                                         float *a, float *b) {
#if CONFIG_BALANCER_TILT_ACCEL_AXIS_X
    *a = y; *b = z;
#elif CONFIG_BALANCER_TILT_ACCEL_AXIS_Z
    *a = x; *b = y;
#else
    *a = x; *b = z;
#endif
}
static inline float pick_gyro_axis(float x, float y, float z) {
#if CONFIG_BALANCER_TILT_GYRO_AXIS_Y
    float v = y;
#elif CONFIG_BALANCER_TILT_GYRO_AXIS_Z
    float v = z;
#else
    float v = x;
#endif
#if CONFIG_BALANCER_TILT_GYRO_INVERT
    v = -v;
#endif
    return v;
}

// ---------- Shared state (guarded by critical section) ----------

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static balancer_status_t s_status;
static volatile bool s_arm_request;
static volatile bool s_disarm_request;

static void set_state(balancer_state_t st) __attribute__((unused));
static void set_state(balancer_state_t st) {
    portENTER_CRITICAL(&s_lock);
    s_status.state = st;
    portEXIT_CRITICAL(&s_lock);
}

// ---------- PID ----------

typedef struct {
    float kp, ki, kd;
    float dir;
    float setpoint;
    float integral;
    float integral_min, integral_max;
    float out_min, out_max;
    float input_last;
    bool  has_last;
} pid_ctrl_t;

static void pid_init(pid_ctrl_t *p, float kp, float ki, float kd, float dir,
                     float out_abs, float integral_abs) {
    memset(p, 0, sizeof(*p));
    p->kp = kp; p->ki = ki; p->kd = kd;
    p->dir = (dir >= 0) ? +1.0f : -1.0f;
    p->out_min = -out_abs; p->out_max = out_abs;
    p->integral_min = -integral_abs; p->integral_max = integral_abs;
}

static void pid_reset(pid_ctrl_t *p) {
    p->integral = 0;
    p->has_last = false;
    p->input_last = 0;
}

static float pid_update(pid_ctrl_t *p, float input) {
    float error = p->setpoint - input;
    p->integral += p->ki * error;
    if (p->integral > p->integral_max) p->integral = p->integral_max;
    if (p->integral < p->integral_min) p->integral = p->integral_min;
    float dinput = p->has_last ? (input - p->input_last) : 0.0f;
    p->input_last = input;
    p->has_last = true;
    float out = p->dir * (p->kp * error + p->integral - p->kd * dinput);
    if (out > p->out_max) out = p->out_max;
    if (out < p->out_min) out = p->out_min;
    return out;
}

// ---------- Control task ----------

static void control_task(void *arg)
{
    ESP_LOGI(TAG, "control task started on core %d", xPortGetCoreID());

    const float nominal_dt_s = 1.0f / (float)CONFIG_BALANCER_LOOP_HZ;
    const TickType_t period  = pdMS_TO_TICKS(1000 / CONFIG_BALANCER_LOOP_HZ);
    if (period == 0) {
        ESP_LOGE(TAG, "loop period < 1 tick; tick rate too low");
        vTaskDelete(NULL);
        return;
    }

    pid_ctrl_t angle_pid, speed_pid;
    const float pwm_abs = (float)CONFIG_BALANCER_PWM_LIMIT;
    pid_init(&angle_pid, kp_angle(), ki_angle(), kd_angle(),
             angle_dir(), pwm_abs, /*integral_abs*/ pwm_abs);
    pid_init(&speed_pid, kp_speed(), ki_speed(), kd_speed(),
             speed_dir(), pwm_abs, (float)CONFIG_BALANCER_SPEED_I_LIMIT);
    angle_pid.setpoint = setpoint_deg();
    speed_pid.setpoint = 0.0f;

    // Complementary filter state
    float angle_deg = 0;
    bool  filter_primed = false;

    // Gyro bias (learned while DISARMED, frozen while ARMING/ARMED).
    // MPU6886 spec bias is ~1-5 dps but we observed ~160 dps on this unit
    // without calibration, which caused the angle to integrate wildly.
    float gyro_bias_dps = 0;
    bool  bias_primed = false;

    // Encoder / speed state (unsigned-safe delta)
    uint32_t enc_l_u = 0, enc_r_u = 0;
    bool enc_primed = false;
    float motor_speed_filt = 0; // filtered (L+R) encoder delta

    // Arming state
    uint32_t arm_still_loops = 0;
    const uint32_t arm_still_target =
        (uint32_t)((CONFIG_BALANCER_ARM_STILL_MS * CONFIG_BALANCER_LOOP_HZ) / 1000);

    int64_t last_us = esp_timer_get_time();
    TickType_t next_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&next_wake, period);

        // --- Measure dt ---
        int64_t now_us = esp_timer_get_time();
        float dt_s = (now_us - last_us) / 1e6f;
        last_us = now_us;
        if (dt_s <= 0 || dt_s > 0.1f) dt_s = nominal_dt_s;
        bool overran = dt_s > (nominal_dt_s * 1.75f);

        // --- Read IMU (one coherent snapshot) ---
        bala2_base_lock_take();
        M5.Imu.update();
        auto imu = M5.Imu.getImuData();
        bala2_base_lock_give();

        float ax = imu.accel.x, ay = imu.accel.y, az = imu.accel.z;
        float gx = imu.gyro.x,  gy = imu.gyro.y,  gz = imu.gyro.z;

        float accel_mag = sqrtf(ax*ax + ay*ay + az*az);

        // Tilt from accel: atan2(selected, |other|)
        float a_tilt = pick_accel_axis(ax, ay, az);
        float a_other_a, a_other_b;
        pick_other_accel_axes(ax, ay, az, &a_other_a, &a_other_b);
        float accel_angle_deg =
            atan2f(a_tilt, sqrtf(a_other_a*a_other_a + a_other_b*a_other_b)) * RAD2DEG;
        float gyro_raw_dps = pick_gyro_axis(gx, gy, gz);
        if (!bias_primed) {
            gyro_bias_dps = gyro_raw_dps;
            bias_primed = true;
        }
        float gyro_dps = gyro_raw_dps - gyro_bias_dps;

        // Complementary filter. If |accel| is far from 1 g (heavy
        // lateral/vertical acceleration), trust the gyro only.
        bool accel_trustworthy =
            (accel_mag > 0.70f && accel_mag < 1.30f);
        if (!filter_primed) {
            angle_deg = accel_angle_deg;
            filter_primed = true;
        } else {
            float gyro_integrated = angle_deg + gyro_dps * dt_s;
            if (accel_trustworthy) {
                angle_deg = 0.98f * gyro_integrated + 0.02f * accel_angle_deg;
            } else {
                angle_deg = gyro_integrated;
            }
        }

        // --- Read encoders ---
        int32_t enc_l = 0, enc_r = 0;
        esp_err_t enc_err = bala2_base_get_encoders(&enc_l, &enc_r);
        int32_t delta_sum = 0;
        if (enc_err == ESP_OK) {
            uint32_t l_u = (uint32_t)enc_l, r_u = (uint32_t)enc_r;
            if (enc_primed) {
                int32_t dl = (int32_t)(l_u - enc_l_u);
                int32_t dr = (int32_t)(r_u - enc_r_u);
                delta_sum = dl + dr;
            }
            enc_l_u = l_u; enc_r_u = r_u; enc_primed = true;
        }
        motor_speed_filt = 0.8f * motor_speed_filt + 0.2f * (float)delta_sum;

        // --- State machine ---
        balancer_state_t st;
        portENTER_CRITICAL(&s_lock);
        st = s_status.state;
        portEXIT_CRITICAL(&s_lock);

        // Learn gyro bias continuously while idle; freeze while armed.
        // ~0.5 s convergence @ 200 Hz.
        if (st == BALANCER_DISARMED ||
            st == BALANCER_CRASHED ||
            st == BALANCER_FAULT) {
            const float alpha = 0.01f;
            gyro_bias_dps = (1.0f - alpha) * gyro_bias_dps +
                                   alpha  * gyro_raw_dps;
            // And don't let the angle integrate away while idle —
            // just show the accel reading (still useful for axis check).
            if (accel_trustworthy) angle_deg = accel_angle_deg;
        }

        // Operator requests
        if (s_disarm_request) {
            s_disarm_request = false;
            s_arm_request = false;
            st = BALANCER_DISARMED;
            pid_reset(&angle_pid);
            pid_reset(&speed_pid);
            arm_still_loops = 0;
        }
        if (s_arm_request && (st == BALANCER_DISARMED ||
                              st == BALANCER_CRASHED ||
                              st == BALANCER_FAULT)) {
            s_arm_request = false;
            st = BALANCER_ARMING;
            arm_still_loops = 0;
            pid_reset(&angle_pid);
            pid_reset(&speed_pid);
            // Pre-load filter from current accel snapshot.
            if (accel_trustworthy) angle_deg = accel_angle_deg;
        }

        // Safety cutoff
        if ((st == BALANCER_ARMED || st == BALANCER_ARMING) &&
            fabsf(angle_deg) > (float)CONFIG_BALANCER_TILT_CUTOFF_DEG) {
            ESP_LOGW(TAG, "tilt cutoff: %.1f deg", angle_deg);
            st = BALANCER_CRASHED;
        }
        // I²C fault -> disarm (crash-safe)
        if (enc_err != ESP_OK && (st == BALANCER_ARMED || st == BALANCER_ARMING)) {
            ESP_LOGW(TAG, "encoder read fault, disarming");
            st = BALANCER_FAULT;
        }
        // Loop overrun -> disarm
        if (overran && st == BALANCER_ARMED) {
            ESP_LOGW(TAG, "loop overrun (dt=%.1f ms), disarming", dt_s * 1000);
            st = BALANCER_FAULT;
        }

        // --- Compute output ---
        int16_t pwm_left = 0, pwm_right = 0;
        if (st == BALANCER_ARMED) {
            float pwm_angle = pid_update(&angle_pid, angle_deg);
            float pwm_speed = pid_update(&speed_pid, motor_speed_filt);
            float pwm = pwm_angle + pwm_speed;
            if (pwm >  pwm_abs) pwm =  pwm_abs;
            if (pwm < -pwm_abs) pwm = -pwm_abs;
            pwm_left  = (int16_t)pwm;
            pwm_right = (int16_t)pwm;
        } else if (st == BALANCER_ARMING) {
            bool within_window = fabsf(angle_deg - angle_pid.setpoint) <
                                 (float)CONFIG_BALANCER_ARM_ANGLE_DEG;
            bool still = fabsf(gyro_dps) < 30.0f && accel_trustworthy;
            if (within_window && still) {
                arm_still_loops++;
                if (arm_still_loops >= arm_still_target) {
                    ESP_LOGI(TAG, "armed @ angle=%.1f", angle_deg);
                    st = BALANCER_ARMED;
                    arm_still_loops = 0;
                    pid_reset(&angle_pid);
                    pid_reset(&speed_pid);
                    // Clear base's encoders so speed loop starts at 0.
                    bala2_base_clear_encoders();
                    enc_primed = false;
                    motor_speed_filt = 0;
                }
            } else {
                arm_still_loops = 0;
            }
        }

        // Apply motor polarity
#if CONFIG_BALANCER_MOTOR_INVERT_LEFT
        pwm_left = -pwm_left;
#endif
#if CONFIG_BALANCER_MOTOR_INVERT_RIGHT
        pwm_right = -pwm_right;
#endif

        esp_err_t mot_err = bala2_base_set_speed(pwm_left, pwm_right);
        if (mot_err != ESP_OK && st == BALANCER_ARMED) {
            ESP_LOGW(TAG, "motor write fault, disarming");
            st = BALANCER_FAULT;
            bala2_base_set_speed(0, 0);
            pwm_left = pwm_right = 0;
        }

        // --- Publish telemetry ---
        portENTER_CRITICAL(&s_lock);
        s_status.state          = st;
        s_status.angle_deg      = angle_deg;
        s_status.gyro_dps       = gyro_dps;
        s_status.accel_mag_g    = accel_mag;
        s_status.pwm_left       = pwm_left;
        s_status.pwm_right      = pwm_right;
        s_status.enc_left       = enc_l;
        s_status.enc_right      = enc_r;
        s_status.loop_dt_ms     = dt_s * 1000.0f;
        s_status.loops++;
        if (overran)              s_status.overrun_count++;
        if (enc_err != ESP_OK ||
            mot_err != ESP_OK)    s_status.i2c_err_count++;
        portEXIT_CRITICAL(&s_lock);
    }
}

extern "C" esp_err_t balancer_start(void)
{
    memset(&s_status, 0, sizeof(s_status));
    s_status.state = BALANCER_DISARMED;
    BaseType_t ok = xTaskCreatePinnedToCore(
        control_task, "balancer", 6 * 1024, NULL,
        /*priority*/ 10, NULL, /*core*/ 1);
    return (ok == pdPASS) ? ESP_OK : ESP_FAIL;
}

extern "C" void balancer_arm(void)    { s_arm_request = true; }
extern "C" void balancer_disarm(void) { s_disarm_request = true; }

extern "C" bool balancer_is_armed(void)
{
    balancer_state_t st;
    portENTER_CRITICAL(&s_lock);
    st = s_status.state;
    portEXIT_CRITICAL(&s_lock);
    return st == BALANCER_ARMED;
}

extern "C" void balancer_get_status(balancer_status_t *out)
{
    if (!out) return;
    portENTER_CRITICAL(&s_lock);
    *out = s_status;
    portEXIT_CRITICAL(&s_lock);
}
