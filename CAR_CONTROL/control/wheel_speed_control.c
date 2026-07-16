#include "wheel_speed_control.h"

#include "board_wheel_drive.h"
#include "encoder_input.h"
#include "pid.h"

#include <stddef.h>

#define WHEEL_SPEED_KP                  0.12f
#define WHEEL_SPEED_KI                  0.05f
#define WHEEL_SPEED_KD                  0.0f
#define WHEEL_SPEED_I_LIMIT             5000.0f
#define WHEEL_SPEED_DEADBAND_PPS        12.0f
#define WHEEL_SPEED_FEEDFORWARD_OFFSET  487.0f
#define WHEEL_SPEED_FEEDFORWARD_GAIN      0.031f
#define WHEEL_SPEED_TARGET_SLEW_PPS_PER_S 2000.0f
#define WHEEL_SPEED_DEFAULT_OUTPUT_MAX  1000U

static pid_controller_t g_left_pid;
static pid_controller_t g_right_pid;
static wheel_speed_control_tunings_t g_tunings;
static wheel_speed_control_snapshot_t g_snapshot;
static uint16_t g_left_output_limit;
static uint16_t g_right_output_limit;
static uint32_t g_last_update_ms;
static float g_left_requested_pps;
static float g_right_requested_pps;

static bool wheel_speed_control_target_is_valid(float target_pps);
static float wheel_speed_control_update_one(pid_controller_t *pid,
    float target_pps, float measured_pps, float dt_s,
    uint16_t output_limit);
static int16_t wheel_speed_control_round_output(float output);
static float wheel_speed_control_slew_target(
    float current, float requested, float max_delta);
static bool wheel_speed_control_crossed_zero(float previous, float current);
static void wheel_speed_control_reset_pid_state(void);
static void wheel_speed_control_deactivate(void);
static void wheel_speed_control_fault(wheel_speed_control_result_t result);

void WheelSpeedControl_Init(uint32_t now_ms)
{
    PID_Init(&g_left_pid);
    PID_Init(&g_right_pid);
    PID_SetTunings(&g_left_pid,
        WHEEL_SPEED_KP, WHEEL_SPEED_KI, WHEEL_SPEED_KD);
    PID_SetTunings(&g_right_pid,
        WHEEL_SPEED_KP, WHEEL_SPEED_KI, WHEEL_SPEED_KD);
    g_tunings.kp = WHEEL_SPEED_KP;
    g_tunings.ki = WHEEL_SPEED_KI;
    g_tunings.kd = WHEEL_SPEED_KD;
    PID_SetIntegralLimits(&g_left_pid, -WHEEL_SPEED_I_LIMIT,
        WHEEL_SPEED_I_LIMIT);
    PID_SetIntegralLimits(&g_right_pid, -WHEEL_SPEED_I_LIMIT,
        WHEEL_SPEED_I_LIMIT);
    PID_SetDeadband(&g_left_pid, WHEEL_SPEED_DEADBAND_PPS);
    PID_SetDeadband(&g_right_pid, WHEEL_SPEED_DEADBAND_PPS);

    g_left_output_limit = WHEEL_SPEED_DEFAULT_OUTPUT_MAX;
    g_right_output_limit = WHEEL_SPEED_DEFAULT_OUTPUT_MAX;
    PID_SetOutputLimits(&g_left_pid, -(float) g_left_output_limit,
        (float) g_left_output_limit);
    PID_SetOutputLimits(&g_right_pid, -(float) g_right_output_limit,
        (float) g_right_output_limit);

    g_snapshot.left_target_pps = 0.0f;
    g_snapshot.right_target_pps = 0.0f;
    g_snapshot.left_error_pps = 0.0f;
    g_snapshot.right_error_pps = 0.0f;
    g_snapshot.left_measured_pps = 0;
    g_snapshot.right_measured_pps = 0;
    g_snapshot.left_output_permille = 0;
    g_snapshot.right_output_permille = 0;
    g_snapshot.update_count = 0U;
    g_snapshot.last_result = WHEEL_SPEED_CONTROL_OK;
    g_snapshot.running = false;
    g_left_requested_pps = 0.0f;
    g_right_requested_pps = 0.0f;
    g_last_update_ms = now_ms;
}

bool WheelSpeedControl_TuningsAreValid(
    const wheel_speed_control_tunings_t *tunings)
{
    if (tunings == NULL) {
        return false;
    }

    return (tunings->kp >= 0.0f) &&
        (tunings->kp <= WHEEL_SPEED_CONTROL_KP_MAX) &&
        (tunings->ki >= 0.0f) &&
        (tunings->ki <= WHEEL_SPEED_CONTROL_KI_MAX) &&
        (tunings->kd >= 0.0f) &&
        (tunings->kd <= WHEEL_SPEED_CONTROL_KD_MAX);
}

wheel_speed_control_result_t WheelSpeedControl_SetTunings(
    const wheel_speed_control_tunings_t *tunings)
{
    if (!WheelSpeedControl_TuningsAreValid(tunings)) {
        g_snapshot.last_result = WHEEL_SPEED_CONTROL_BAD_TUNING;
        return g_snapshot.last_result;
    }
    if (g_snapshot.running) {
        g_snapshot.last_result = WHEEL_SPEED_CONTROL_BUSY;
        return g_snapshot.last_result;
    }

    g_tunings = *tunings;
    PID_SetTunings(&g_left_pid,
        g_tunings.kp, g_tunings.ki, g_tunings.kd);
    PID_SetTunings(&g_right_pid,
        g_tunings.kp, g_tunings.ki, g_tunings.kd);
    wheel_speed_control_reset_pid_state();
    g_snapshot.last_result = WHEEL_SPEED_CONTROL_OK;
    return WHEEL_SPEED_CONTROL_OK;
}

bool WheelSpeedControl_GetTunings(
    wheel_speed_control_tunings_t *tunings)
{
    if (tunings == NULL) {
        return false;
    }
    *tunings = g_tunings;
    return true;
}

wheel_speed_control_result_t WheelSpeedControl_Start(uint32_t now_ms)
{
    if (g_snapshot.running) {
        return WHEEL_SPEED_CONTROL_OK;
    }
    if (ControlSupervisor_BeginSpeedControl(now_ms) !=
        CAR_CONTROL_REQUEST_OK) {
        g_snapshot.last_result =
            WHEEL_SPEED_CONTROL_SUPERVISOR_BLOCKED;
        return g_snapshot.last_result;
    }

    wheel_speed_control_reset_pid_state();
    g_snapshot.left_target_pps = 0.0f;
    g_snapshot.right_target_pps = 0.0f;
    g_snapshot.left_error_pps = 0.0f;
    g_snapshot.right_error_pps = 0.0f;
    g_snapshot.left_measured_pps = 0;
    g_snapshot.right_measured_pps = 0;
    g_snapshot.left_output_permille = 0;
    g_snapshot.right_output_permille = 0;
    g_snapshot.update_count = 0U;
    g_snapshot.last_result = WHEEL_SPEED_CONTROL_OK;
    g_snapshot.running = true;
    g_left_requested_pps = 0.0f;
    g_right_requested_pps = 0.0f;
    g_last_update_ms = now_ms;
    return WHEEL_SPEED_CONTROL_OK;
}

wheel_speed_control_result_t WheelSpeedControl_SetTargets(
    float left_pps, float right_pps)
{
    if (!g_snapshot.running) {
        g_snapshot.last_result = WHEEL_SPEED_CONTROL_NOT_RUNNING;
        return g_snapshot.last_result;
    }
    if (!wheel_speed_control_target_is_valid(left_pps) ||
        !wheel_speed_control_target_is_valid(right_pps)) {
        wheel_speed_control_fault(WHEEL_SPEED_CONTROL_BAD_TARGET);
        return WHEEL_SPEED_CONTROL_BAD_TARGET;
    }

    g_left_requested_pps = left_pps;
    g_right_requested_pps = right_pps;
    if ((left_pps == 0.0f) && (right_pps == 0.0f)) {
        g_snapshot.left_target_pps = 0.0f;
        g_snapshot.right_target_pps = 0.0f;
        wheel_speed_control_reset_pid_state();
        g_snapshot.left_output_permille = 0;
        g_snapshot.right_output_permille = 0;
        if (BoardWheelDrive_SetCommands(0, 0) !=
            BOARD_WHEEL_DRIVE_OK) {
            wheel_speed_control_fault(WHEEL_SPEED_CONTROL_OUTPUT_ERROR);
            return WHEEL_SPEED_CONTROL_OUTPUT_ERROR;
        }
    }

    g_snapshot.last_result = WHEEL_SPEED_CONTROL_OK;
    return WHEEL_SPEED_CONTROL_OK;
}

wheel_speed_control_result_t WheelSpeedControl_SetOutputLimits(
    uint16_t left_permille, uint16_t right_permille)
{
    if ((left_permille == 0U) ||
        (left_permille > BOARD_WHEEL_DRIVE_COMMAND_MAX) ||
        (right_permille == 0U) ||
        (right_permille > BOARD_WHEEL_DRIVE_COMMAND_MAX)) {
        g_snapshot.last_result = WHEEL_SPEED_CONTROL_BAD_OUTPUT_LIMIT;
        return WHEEL_SPEED_CONTROL_BAD_OUTPUT_LIMIT;
    }

    g_left_output_limit = left_permille;
    g_right_output_limit = right_permille;
    PID_SetOutputLimits(&g_left_pid, -(float) g_left_output_limit,
        (float) g_left_output_limit);
    PID_SetOutputLimits(&g_right_pid, -(float) g_right_output_limit,
        (float) g_right_output_limit);
    g_snapshot.last_result = WHEEL_SPEED_CONTROL_OK;
    return WHEEL_SPEED_CONTROL_OK;
}

void WheelSpeedControl_Task(uint32_t now_ms)
{
    encoder_input_snapshot_t left_encoder;
    encoder_input_snapshot_t right_encoder;
    uint32_t elapsed_ms;
    float dt_s;
    float target_max_delta;
    float previous_left_target;
    float previous_right_target;
    float left_output;
    float right_output;

    if (!g_snapshot.running) {
        return;
    }
    if (ControlSupervisor_GetMode() != CAR_CONTROL_MODE_SPEED) {
        g_snapshot.last_result =
            WHEEL_SPEED_CONTROL_SUPERVISOR_BLOCKED;
        wheel_speed_control_deactivate();
        return;
    }

    elapsed_ms = now_ms - g_last_update_ms;
    if (elapsed_ms < ENCODER_INPUT_SAMPLE_INTERVAL_MS) {
        return;
    }
    if (!EncoderInput_GetSnapshot(ENCODER_INPUT_0, &left_encoder) ||
        !EncoderInput_GetSnapshot(ENCODER_INPUT_1, &right_encoder)) {
        wheel_speed_control_fault(WHEEL_SPEED_CONTROL_ENCODER_ERROR);
        return;
    }

    dt_s = (float) elapsed_ms / 1000.0f;
    g_last_update_ms = now_ms;
    target_max_delta = WHEEL_SPEED_TARGET_SLEW_PPS_PER_S * dt_s;
    previous_left_target = g_snapshot.left_target_pps;
    previous_right_target = g_snapshot.right_target_pps;
    g_snapshot.left_target_pps = wheel_speed_control_slew_target(
        previous_left_target, g_left_requested_pps, target_max_delta);
    g_snapshot.right_target_pps = wheel_speed_control_slew_target(
        previous_right_target, g_right_requested_pps, target_max_delta);
    if (wheel_speed_control_crossed_zero(
            previous_left_target, g_snapshot.left_target_pps)) {
        PID_Reset(&g_left_pid);
    }
    if (wheel_speed_control_crossed_zero(
            previous_right_target, g_snapshot.right_target_pps)) {
        PID_Reset(&g_right_pid);
    }
    g_snapshot.left_measured_pps = left_encoder.speed_pps;
    g_snapshot.right_measured_pps = right_encoder.speed_pps;
    g_snapshot.left_error_pps = g_snapshot.left_target_pps -
        (float) g_snapshot.left_measured_pps;
    g_snapshot.right_error_pps = g_snapshot.right_target_pps -
        (float) g_snapshot.right_measured_pps;

    left_output = wheel_speed_control_update_one(&g_left_pid,
        g_snapshot.left_target_pps,
        (float) g_snapshot.left_measured_pps, dt_s,
        g_left_output_limit);
    right_output = wheel_speed_control_update_one(&g_right_pid,
        g_snapshot.right_target_pps,
        (float) g_snapshot.right_measured_pps, dt_s,
        g_right_output_limit);
    g_snapshot.left_output_permille =
        wheel_speed_control_round_output(left_output);
    g_snapshot.right_output_permille =
        wheel_speed_control_round_output(right_output);

    if (BoardWheelDrive_SetCommands(
            g_snapshot.left_output_permille,
            g_snapshot.right_output_permille) != BOARD_WHEEL_DRIVE_OK) {
        wheel_speed_control_fault(WHEEL_SPEED_CONTROL_OUTPUT_ERROR);
        return;
    }
    if (ControlSupervisor_RefreshSpeedControl(now_ms) !=
        CAR_CONTROL_REQUEST_OK) {
        wheel_speed_control_fault(
            WHEEL_SPEED_CONTROL_SUPERVISOR_BLOCKED);
        return;
    }

    g_snapshot.update_count++;
    g_snapshot.last_result = WHEEL_SPEED_CONTROL_OK;
}

void WheelSpeedControl_Stop(car_control_block_reason_t reason)
{
    wheel_speed_control_deactivate();
    g_snapshot.last_result = WHEEL_SPEED_CONTROL_OK;
    ControlSupervisor_EmergencyStop(reason);
}

bool WheelSpeedControl_GetSnapshot(
    wheel_speed_control_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return false;
    }
    *snapshot = g_snapshot;
    return true;
}

static bool wheel_speed_control_target_is_valid(float target_pps)
{
    return (target_pps >= -WHEEL_SPEED_CONTROL_TARGET_MAX_PPS) &&
        (target_pps <= WHEEL_SPEED_CONTROL_TARGET_MAX_PPS);
}

static float wheel_speed_control_update_one(pid_controller_t *pid,
    float target_pps, float measured_pps, float dt_s,
    uint16_t output_limit)
{
    float feedforward;
    float correction;
    float target_magnitude;

    if (target_pps == 0.0f) {
        PID_Reset(pid);
        return 0.0f;
    }

    target_magnitude = (target_pps > 0.0f) ? target_pps : -target_pps;
    feedforward = WHEEL_SPEED_FEEDFORWARD_OFFSET +
        WHEEL_SPEED_FEEDFORWARD_GAIN * target_magnitude;
    if (feedforward > (float) output_limit) {
        feedforward = (float) output_limit;
    }

    if (target_pps > 0.0f) {
        PID_SetOutputLimits(pid, -feedforward,
            (float) output_limit - feedforward);
    } else {
        feedforward = -feedforward;
        PID_SetOutputLimits(pid,
            -(float) output_limit - feedforward, -feedforward);
    }

    correction = PID_Update(pid, target_pps, measured_pps, dt_s);
    return feedforward + correction;
}

static int16_t wheel_speed_control_round_output(float output)
{
    if (output >= 0.0f) {
        return (int16_t) (output + 0.5f);
    }
    return (int16_t) (output - 0.5f);
}

static float wheel_speed_control_slew_target(
    float current, float requested, float max_delta)
{
    if (requested > (current + max_delta)) {
        return current + max_delta;
    }
    if (requested < (current - max_delta)) {
        return current - max_delta;
    }
    return requested;
}

static bool wheel_speed_control_crossed_zero(float previous, float current)
{
    return ((previous > 0.0f) && (current <= 0.0f)) ||
        ((previous < 0.0f) && (current >= 0.0f));
}

static void wheel_speed_control_reset_pid_state(void)
{
    PID_Reset(&g_left_pid);
    PID_Reset(&g_right_pid);
}

static void wheel_speed_control_deactivate(void)
{
    g_snapshot.running = false;
    g_snapshot.left_target_pps = 0.0f;
    g_snapshot.right_target_pps = 0.0f;
    g_snapshot.left_error_pps = 0.0f;
    g_snapshot.right_error_pps = 0.0f;
    g_snapshot.left_output_permille = 0;
    g_snapshot.right_output_permille = 0;
    g_left_requested_pps = 0.0f;
    g_right_requested_pps = 0.0f;
    wheel_speed_control_reset_pid_state();
    BoardWheelDrive_SetZero();
}

static void wheel_speed_control_fault(wheel_speed_control_result_t result)
{
    wheel_speed_control_deactivate();
    g_snapshot.last_result = result;
    ControlSupervisor_EmergencyStop(CAR_CONTROL_BLOCK_EMERGENCY_STOP);
}
