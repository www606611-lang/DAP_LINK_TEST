#include "encoder_speed_test.h"

#include "encoder_motor_pid.h"
#include "encoder.h"
#include "motor.h"
#include "uart0_dma.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define TEST_START_DELAY_MS          100U
#define TEST_STEP_INTERVAL_MS        3000U
#define TEST_TELEMETRY_MS            50U
#define TEST_LEFT_SPEED_KP           0.08f
#define TEST_LEFT_SPEED_KI           0.0f
#define TEST_LEFT_SPEED_KD           0.0f
#define TEST_RIGHT_SPEED_KP          0.08f
#define TEST_RIGHT_SPEED_KI          0.0f
#define TEST_RIGHT_SPEED_KD          0.0f
#define TEST_SPEED_I_LIMIT           120.0f
#define TEST_SPEED_DEADBAND          24.0f
#define TEST_SPEED_PWM_LIMIT         420.0f
#define TEST_TARGET_RAMP_PPS_PER_S   200.0f
#define TEST_MOTOR_AUTO_RUN          1

static const float g_encoder_speed_test_targets_pps[] = {
    300.0f, 500.0f, 700.0f, 500.0f
};

static uint32_t g_encoder_speed_test_boot_ms;
static uint32_t g_encoder_speed_test_last_step_ms;
static uint32_t g_encoder_speed_test_last_telemetry_ms;
static uint32_t g_encoder_speed_test_last_ramp_ms;
static bool g_encoder_speed_test_armed;
static bool g_encoder_speed_test_started;
static uint8_t g_encoder_speed_test_target_index;
static float g_encoder_speed_test_requested_target_pps;
static float g_encoder_speed_test_active_target_pps;
static encoder_motor_pid_t g_left_speed_loop;
static encoder_motor_pid_t g_right_speed_loop;
static char g_encoder_speed_test_uart_line[128];

static void encoder_speed_test_configure_loop(encoder_motor_pid_t *loop,
    motor_id_t motor_id, encoder_id_t encoder_id, uint32_t now_ms);
static void encoder_speed_test_start(uint32_t now_ms);
static void encoder_speed_test_apply_target(float target_speed_pps);
static void encoder_speed_test_step_target(uint32_t now_ms);
static void encoder_speed_test_update_ramped_target(uint32_t now_ms);
static void encoder_speed_test_send_telemetry(uint32_t now_ms);
static float encoder_speed_test_move_toward(
    float current, float target, float max_delta);
#if TEST_MOTOR_AUTO_RUN
static int32_t encoder_speed_test_round_float(float value);
#endif

void encoder_speed_test_init(uint32_t now_ms)
{
    Encoder_SetInverted(ENCODER_LEFT, true);
    Encoder_ResetAll();
    Motor_SetRightInverted(true);

    encoder_speed_test_configure_loop(
        &g_left_speed_loop, MOTOR_LEFT, ENCODER_LEFT, now_ms);
    encoder_speed_test_configure_loop(
        &g_right_speed_loop, MOTOR_RIGHT, ENCODER_RIGHT, now_ms);

    Motor_Stop();
    g_encoder_speed_test_boot_ms = now_ms;
    g_encoder_speed_test_last_step_ms = now_ms;
    g_encoder_speed_test_last_telemetry_ms = now_ms;
    g_encoder_speed_test_last_ramp_ms = now_ms;
    g_encoder_speed_test_armed = false;
    g_encoder_speed_test_started = false;
    g_encoder_speed_test_target_index = 0U;
    g_encoder_speed_test_requested_target_pps = 0.0f;
    g_encoder_speed_test_active_target_pps = 0.0f;
}

void encoder_speed_test_task(uint32_t now_ms)
{
#if TEST_MOTOR_AUTO_RUN
    if (!g_encoder_speed_test_armed) {
        g_encoder_speed_test_boot_ms = now_ms;
        g_encoder_speed_test_last_step_ms = now_ms;
        g_encoder_speed_test_last_telemetry_ms = now_ms;
        g_encoder_speed_test_last_ramp_ms = now_ms;
        g_encoder_speed_test_armed = true;
        return;
    }

    if (g_encoder_speed_test_started) {
        encoder_speed_test_update_ramped_target(now_ms);
        (void) EncoderMotorPID_Update(&g_left_speed_loop, now_ms);
        (void) EncoderMotorPID_Update(&g_right_speed_loop, now_ms);
        encoder_speed_test_step_target(now_ms);
        encoder_speed_test_send_telemetry(now_ms);
        return;
    }

    if ((uint32_t) (now_ms - g_encoder_speed_test_boot_ms) >=
        TEST_START_DELAY_MS) {
        encoder_speed_test_start(now_ms);
    }
#else
    (void) g_encoder_speed_test_boot_ms;
    (void) g_encoder_speed_test_last_step_ms;
    (void) g_encoder_speed_test_last_ramp_ms;
    (void) g_encoder_speed_test_armed;
    (void) g_encoder_speed_test_started;
    (void) g_encoder_speed_test_target_index;
    (void) g_encoder_speed_test_requested_target_pps;
    (void) g_encoder_speed_test_active_target_pps;
    (void) g_encoder_speed_test_targets_pps;
    (void) encoder_speed_test_start;
    (void) encoder_speed_test_apply_target;
    (void) encoder_speed_test_step_target;
    (void) encoder_speed_test_update_ramped_target;
    (void) encoder_speed_test_move_toward;

    encoder_speed_test_send_telemetry(now_ms);
#endif
}

static void encoder_speed_test_configure_loop(encoder_motor_pid_t *loop,
    motor_id_t motor_id, encoder_id_t encoder_id, uint32_t now_ms)
{
    EncoderMotorPID_Init(loop, motor_id, encoder_id, now_ms);
    if (motor_id == MOTOR_LEFT) {
        EncoderMotorPID_SetSpeedTunings(
            loop, TEST_LEFT_SPEED_KP, TEST_LEFT_SPEED_KI, TEST_LEFT_SPEED_KD);
    } else {
        EncoderMotorPID_SetSpeedTunings(
            loop, TEST_RIGHT_SPEED_KP, TEST_RIGHT_SPEED_KI,
            TEST_RIGHT_SPEED_KD);
    }
    EncoderMotorPID_SetSpeedOutputLimits(
        loop, -TEST_SPEED_PWM_LIMIT, TEST_SPEED_PWM_LIMIT);
    EncoderMotorPID_SetSpeedIntegralLimits(
        loop, -TEST_SPEED_I_LIMIT, TEST_SPEED_I_LIMIT);
    EncoderMotorPID_SetSpeedDeadband(loop, TEST_SPEED_DEADBAND);
}

static void encoder_speed_test_start(uint32_t now_ms)
{
    g_encoder_speed_test_started = true;
    Encoder_ResetAll();
    EncoderMotorPID_Reset(&g_left_speed_loop, now_ms);
    EncoderMotorPID_Reset(&g_right_speed_loop, now_ms);
    g_encoder_speed_test_target_index = 0U;
    g_encoder_speed_test_last_step_ms = now_ms;
    g_encoder_speed_test_last_telemetry_ms = now_ms;
    g_encoder_speed_test_last_ramp_ms = now_ms;
    g_encoder_speed_test_active_target_pps = 0.0f;
    g_encoder_speed_test_requested_target_pps =
        g_encoder_speed_test_targets_pps[g_encoder_speed_test_target_index];
    encoder_speed_test_apply_target(g_encoder_speed_test_active_target_pps);
}

static void encoder_speed_test_apply_target(float target_speed_pps)
{
    EncoderMotorPID_SetTargetSpeedPps(&g_left_speed_loop, target_speed_pps);
    EncoderMotorPID_SetTargetSpeedPps(&g_right_speed_loop, target_speed_pps);
}

static void encoder_speed_test_step_target(uint32_t now_ms)
{
    if ((uint32_t) (now_ms - g_encoder_speed_test_last_step_ms) <
        TEST_STEP_INTERVAL_MS) {
        return;
    }

    g_encoder_speed_test_last_step_ms = now_ms;
    g_encoder_speed_test_target_index =
        (uint8_t) ((g_encoder_speed_test_target_index + 1U) %
        (sizeof(g_encoder_speed_test_targets_pps) /
            sizeof(g_encoder_speed_test_targets_pps[0])));
    g_encoder_speed_test_requested_target_pps =
        g_encoder_speed_test_targets_pps[g_encoder_speed_test_target_index];
}

static void encoder_speed_test_update_ramped_target(uint32_t now_ms)
{
    uint32_t elapsed_ms = now_ms - g_encoder_speed_test_last_ramp_ms;
    float max_delta;
    float next_target;

    if (elapsed_ms == 0U) {
        return;
    }

    g_encoder_speed_test_last_ramp_ms = now_ms;
    max_delta = (TEST_TARGET_RAMP_PPS_PER_S * (float) elapsed_ms) / 1000.0f;
    next_target = encoder_speed_test_move_toward(
        g_encoder_speed_test_active_target_pps,
        g_encoder_speed_test_requested_target_pps,
        max_delta);

    if (next_target != g_encoder_speed_test_active_target_pps) {
        g_encoder_speed_test_active_target_pps = next_target;
        encoder_speed_test_apply_target(g_encoder_speed_test_active_target_pps);
    }
}

static void encoder_speed_test_send_telemetry(uint32_t now_ms)
{
    encoder_snapshot_t left_snapshot;
    encoder_snapshot_t right_snapshot;
    int32_t left_target;
    int32_t right_target;
    int32_t left_pwm;
    int32_t right_pwm;
    int length;

    if ((uint32_t) (now_ms - g_encoder_speed_test_last_telemetry_ms) <
        TEST_TELEMETRY_MS) {
        return;
    }
    if (uart0_dma_tx_busy()) {
        return;
    }
    if (!EncoderMotorPID_GetSnapshot(&g_left_speed_loop, &left_snapshot) ||
        !EncoderMotorPID_GetSnapshot(&g_right_speed_loop, &right_snapshot)) {
        return;
    }

#if !TEST_MOTOR_AUTO_RUN
    if (!Encoder_GetSnapshot(ENCODER_LEFT, &left_snapshot) ||
        !Encoder_GetSnapshot(ENCODER_RIGHT, &right_snapshot)) {
        return;
    }
#endif

    g_encoder_speed_test_last_telemetry_ms = now_ms;
#if TEST_MOTOR_AUTO_RUN
    left_target = encoder_speed_test_round_float(
        EncoderMotorPID_GetSpeedTargetPps(&g_left_speed_loop));
    right_target = encoder_speed_test_round_float(
        EncoderMotorPID_GetSpeedTargetPps(&g_right_speed_loop));
    left_pwm = encoder_speed_test_round_float(
        EncoderMotorPID_GetPwmCommand(&g_left_speed_loop));
    right_pwm = encoder_speed_test_round_float(
        EncoderMotorPID_GetPwmCommand(&g_right_speed_loop));
#else
    left_target = 0;
    right_target = 0;
    left_pwm = 0;
    right_pwm = 0;
#endif
    length = snprintf(
        g_encoder_speed_test_uart_line,
        sizeof(g_encoder_speed_test_uart_line),
        "d:%ld,%ld,%ld,%ld,%ld,%ld\n",
        (long) left_target,
        (long) right_target,
        (long) left_snapshot.speed_pps,
        (long) right_snapshot.speed_pps,
        (long) left_pwm,
        (long) right_pwm);

    if ((length <= 0) ||
        ((size_t) length >= sizeof(g_encoder_speed_test_uart_line))) {
        return;
    }

    (void) uart0_dma_send(
        (const uint8_t *) g_encoder_speed_test_uart_line, (uint16_t) length);
}

#if TEST_MOTOR_AUTO_RUN
static int32_t encoder_speed_test_round_float(float value)
{
    if (value >= 0.0f) {
        return (int32_t) (value + 0.5f);
    }

    return (int32_t) (value - 0.5f);
}
#endif

static float encoder_speed_test_move_toward(
    float current, float target, float max_delta)
{
    if (max_delta <= 0.0f) {
        return current;
    }
    if (target > (current + max_delta)) {
        return current + max_delta;
    }
    if (target < (current - max_delta)) {
        return current - max_delta;
    }
    return target;
}
