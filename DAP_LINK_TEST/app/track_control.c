#include "track_control.h"

#include "k230_uart.h"
#include "pid.h"
#include "zdt_stepper.h"

#include <stdbool.h>
#include <stddef.h>

#define TRACK_CONTROL_ADDR_1         (1U)
#define TRACK_CONTROL_ADDR_2         (2U)
#define TRACK_CONTROL_ACCEL          (15U)
#define TRACK_CONTROL_BOOT_DELAY     (1000U)
#define TRACK_CONTROL_UPDATE_MS      (20U)
#define TRACK_CONTROL_X_RPM_STEP_LIMIT (2)
#define TRACK_CONTROL_Y_RPM_STEP_LIMIT (2)

#define TRACK_CONTROL_X_KP           (0.10f)
#define TRACK_CONTROL_X_KI           (0.0f)
#define TRACK_CONTROL_X_KD           (0.009f)
#define TRACK_CONTROL_Y_KP           (0.08f)
#define TRACK_CONTROL_Y_KI           (0.0f)
#define TRACK_CONTROL_Y_KD           (0.010f)

#define TRACK_CONTROL_X_DEADBAND     (7.0f)
#define TRACK_CONTROL_Y_DEADBAND     (6.0f)
#define TRACK_CONTROL_ZERO_BAND_RPM  (0.2f)
#define TRACK_CONTROL_MIN_RPM        (1.0f)
#define TRACK_CONTROL_X_MAX_RPM      (7.0f)
#define TRACK_CONTROL_Y_MAX_RPM      (5.0f)

#define TRACK_CONTROL_X_SIGN         (1.0f)
#define TRACK_CONTROL_Y_SIGN         (1.0f)

static bool g_track_control_ready;
static uint32_t g_track_control_deadline_ms;
static uint32_t g_track_control_last_update_ms;
static int16_t g_track_control_motor_1_rpm;
static int16_t g_track_control_motor_2_rpm;
static pid_controller_t g_track_control_pid_x;
static pid_controller_t g_track_control_pid_y;

static void track_control_drain_rx(void);
static void track_control_configure_pid(void);
static int16_t track_control_pid_to_rpm(
    float pid_output, float sign, float max_rpm);
static float track_control_abs(float value);
static void track_control_stop_all(void);
static int16_t track_control_limit_rpm_step(
    int16_t target_rpm, int16_t last_rpm, int16_t step_limit);
static void track_control_apply_speed(
    zdt_stepper_id_t motor, int16_t rpm, int16_t *last_rpm);

void track_control_init(uint32_t now_ms)
{
    ZdtStepper_Init();
    ZdtStepper_SetAddress(ZDT_STEPPER_1, TRACK_CONTROL_ADDR_1);
    ZdtStepper_SetAddress(ZDT_STEPPER_2, TRACK_CONTROL_ADDR_2);
    ZdtStepper_SetInverted(ZDT_STEPPER_1, false);
    ZdtStepper_SetInverted(ZDT_STEPPER_2, false);

    track_control_configure_pid();

    g_track_control_ready = false;
    g_track_control_deadline_ms = now_ms + TRACK_CONTROL_BOOT_DELAY;
    g_track_control_last_update_ms = now_ms;
    g_track_control_motor_1_rpm = 0;
    g_track_control_motor_2_rpm = 0;
}

void track_control_task(uint32_t now_ms)
{
    float dt_s;
    k230_uart_target_t target;
    float pid_x;
    float pid_y;
    int16_t motor_1_rpm;
    int16_t motor_2_rpm;

    track_control_drain_rx();

    if (!g_track_control_ready) {
        if ((int32_t) (now_ms - g_track_control_deadline_ms) < 0) {
            return;
        }

        (void) ZdtStepper_EnableAll(true);
        (void) ZdtStepper_ClearStall(ZDT_STEPPER_1);
        (void) ZdtStepper_ClearStall(ZDT_STEPPER_2);
        (void) ZdtStepper_StopAll();
        g_track_control_motor_1_rpm = 0;
        g_track_control_motor_2_rpm = 0;
        PID_Reset(&g_track_control_pid_x);
        PID_Reset(&g_track_control_pid_y);
        g_track_control_ready = true;
        g_track_control_last_update_ms = now_ms;
        return;
    }

    if ((uint32_t) (now_ms - g_track_control_last_update_ms) <
        TRACK_CONTROL_UPDATE_MS) {
        return;
    }

    dt_s = (float) (now_ms - g_track_control_last_update_ms) / 1000.0f;
    g_track_control_last_update_ms = now_ms;
    target = k230_uart_get_target();

    if (!target.valid) {
        PID_Reset(&g_track_control_pid_x);
        PID_Reset(&g_track_control_pid_y);
        track_control_stop_all();
        return;
    }

    pid_x = PID_Update(&g_track_control_pid_x, 0.0f, (float) target.err_x, dt_s);
    pid_y = PID_Update(&g_track_control_pid_y, 0.0f, (float) target.err_y, dt_s);

    motor_1_rpm = track_control_pid_to_rpm(
        pid_x, TRACK_CONTROL_X_SIGN, TRACK_CONTROL_X_MAX_RPM);
    motor_2_rpm = track_control_pid_to_rpm(
        pid_y, TRACK_CONTROL_Y_SIGN, TRACK_CONTROL_Y_MAX_RPM);

    motor_1_rpm = track_control_limit_rpm_step(
        motor_1_rpm, g_track_control_motor_1_rpm,
        TRACK_CONTROL_X_RPM_STEP_LIMIT);
    motor_2_rpm = track_control_limit_rpm_step(
        motor_2_rpm, g_track_control_motor_2_rpm,
        TRACK_CONTROL_Y_RPM_STEP_LIMIT);

    track_control_apply_speed(
        ZDT_STEPPER_1, motor_1_rpm, &g_track_control_motor_1_rpm);
    track_control_apply_speed(
        ZDT_STEPPER_2, motor_2_rpm, &g_track_control_motor_2_rpm);
}

static void track_control_drain_rx(void)
{
    zdt_stepper_can_frame_t frame;

    while (ZdtStepper_ReadFrame(&frame)) {
    }
}

static void track_control_configure_pid(void)
{
    PID_Init(&g_track_control_pid_x);
    PID_Init(&g_track_control_pid_y);

    PID_SetTunings(
        &g_track_control_pid_x, TRACK_CONTROL_X_KP, TRACK_CONTROL_X_KI,
        TRACK_CONTROL_X_KD);
    PID_SetTunings(
        &g_track_control_pid_y, TRACK_CONTROL_Y_KP, TRACK_CONTROL_Y_KI,
        TRACK_CONTROL_Y_KD);

    PID_SetOutputLimits(
        &g_track_control_pid_x, -TRACK_CONTROL_X_MAX_RPM,
        TRACK_CONTROL_X_MAX_RPM);
    PID_SetOutputLimits(
        &g_track_control_pid_y, -TRACK_CONTROL_Y_MAX_RPM,
        TRACK_CONTROL_Y_MAX_RPM);

    PID_SetIntegralLimits(&g_track_control_pid_x, -6.0f, 6.0f);
    PID_SetIntegralLimits(&g_track_control_pid_y, -6.0f, 6.0f);

    PID_SetDeadband(&g_track_control_pid_x, TRACK_CONTROL_X_DEADBAND);
    PID_SetDeadband(&g_track_control_pid_y, TRACK_CONTROL_Y_DEADBAND);
}

static int16_t track_control_pid_to_rpm(
    float pid_output, float sign, float max_rpm)
{
    float command = pid_output * sign;

    if (track_control_abs(command) <= TRACK_CONTROL_ZERO_BAND_RPM) {
        return 0;
    }

    if (command > 0.0f) {
        if (command < TRACK_CONTROL_MIN_RPM) {
            command = TRACK_CONTROL_MIN_RPM;
        }
    } else {
        if (command > -TRACK_CONTROL_MIN_RPM) {
            command = -TRACK_CONTROL_MIN_RPM;
        }
    }

    if (command > max_rpm) {
        command = max_rpm;
    }
    if (command < -max_rpm) {
        command = -max_rpm;
    }

    return (command >= 0.0f) ? (int16_t) (command + 0.5f) :
                               (int16_t) (command - 0.5f);
}

static float track_control_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void track_control_stop_all(void)
{
    (void) ZdtStepper_StopAll();
    g_track_control_motor_1_rpm = 0;
    g_track_control_motor_2_rpm = 0;
}

static int16_t track_control_limit_rpm_step(
    int16_t target_rpm, int16_t last_rpm, int16_t step_limit)
{
    int16_t delta = (int16_t) (target_rpm - last_rpm);

    if (delta > step_limit) {
        return (int16_t) (last_rpm + step_limit);
    }
    if (delta < -step_limit) {
        return (int16_t) (last_rpm - step_limit);
    }

    return target_rpm;
}

static void track_control_apply_speed(
    zdt_stepper_id_t motor, int16_t rpm, int16_t *last_rpm)
{
    if ((last_rpm == NULL) || (rpm == *last_rpm)) {
        return;
    }

    if (rpm == 0) {
        (void) ZdtStepper_Stop(motor, false);
    } else {
        (void) ZdtStepper_SetSpeed(motor, rpm, TRACK_CONTROL_ACCEL);
    }

    *last_rpm = rpm;
}
