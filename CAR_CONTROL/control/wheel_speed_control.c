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
#define WHEEL_SPEED_BOOST_MEASURED_MAX_PPS 100.0f
#define WHEEL_SPEED_TARGET_SLEW_PPS_PER_S          2000.0f
#define WHEEL_SPEED_POSITION_TARGET_SLEW_PPS_PER_S 4000.0f
#define WHEEL_SPEED_YAW_TARGET_SLEW_PPS_PER_S      6000.0f
#define WHEEL_SPEED_HEADING_TARGET_SLEW_PPS_PER_S  4000.0f
#define WHEEL_SPEED_DEFAULT_OUTPUT_MAX  1000U

static pid_controller_t g_left_pid;
static pid_controller_t g_right_pid;
static wheel_speed_control_tunings_t g_tunings;
static wheel_speed_control_snapshot_t g_snapshot;
static uint16_t g_left_output_limit;
static uint16_t g_right_output_limit;
static uint32_t g_last_update_ms;
static uint32_t g_last_command_ms;
static uint32_t g_command_deadline_ms;
static car_control_mode_t g_owner_mode;
static uint16_t g_feedforward_boost_permille;
static float g_feedforward_ramp_pps;
static bool g_left_feedforward_startup_active;
static bool g_right_feedforward_startup_active;
static float g_left_requested_pps;
static float g_right_requested_pps;

static bool wheel_speed_control_target_is_valid(float target_pps);
static float wheel_speed_control_update_one(pid_controller_t *pid,
    float target_pps, float measured_pps, float dt_s,
    uint16_t output_limit, uint16_t feedforward_boost_permille,
    float feedforward_ramp_pps, bool *feedforward_startup_active);
static int16_t wheel_speed_control_round_output(float output);
static float wheel_speed_control_slew_target(
    float current, float requested, float max_delta);
static bool wheel_speed_control_crossed_zero(float previous, float current);
static float wheel_speed_control_target_slew_rate(void);
static bool wheel_speed_control_deadline_reached(
    uint32_t now_ms, uint32_t deadline_ms);
static void wheel_speed_control_reset_pid_state(void);
static void wheel_speed_control_deactivate(void);
static void wheel_speed_control_fault(wheel_speed_control_result_t result,
    car_control_block_reason_t reason);

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
    g_snapshot.command_age_ms = 0U;
    g_snapshot.last_result = WHEEL_SPEED_CONTROL_OK;
    g_snapshot.owner_mode = CAR_CONTROL_MODE_SPEED;
    g_snapshot.running = false;
    g_owner_mode = CAR_CONTROL_MODE_SPEED;
    g_left_requested_pps = 0.0f;
    g_right_requested_pps = 0.0f;
    g_feedforward_boost_permille = 0U;
    g_feedforward_ramp_pps = 0.0f;
    g_left_feedforward_startup_active = false;
    g_right_feedforward_startup_active = false;
    g_last_update_ms = now_ms;
    g_last_command_ms = now_ms;
    g_command_deadline_ms = now_ms;
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
    return WheelSpeedControl_StartForMode(
        CAR_CONTROL_MODE_SPEED, now_ms);
}

wheel_speed_control_result_t WheelSpeedControl_StartForMode(
    car_control_mode_t owner_mode, uint32_t now_ms)
{
    return WheelSpeedControl_StartForModeWithFeedforward(
        owner_mode, 0U, 0.0f, now_ms);
}

wheel_speed_control_result_t WheelSpeedControl_StartForModeWithFeedforward(
    car_control_mode_t owner_mode, uint16_t feedforward_boost_permille,
    float feedforward_ramp_pps, uint32_t now_ms)
{
    if (!ControlSupervisor_ModeCanOwnSpeedControl(owner_mode)) {
        g_snapshot.last_result = WHEEL_SPEED_CONTROL_BAD_OWNER;
        return g_snapshot.last_result;
    }
    if ((feedforward_boost_permille >
            WHEEL_SPEED_CONTROL_FEEDFORWARD_BOOST_MAX) ||
        !(feedforward_ramp_pps >= 0.0f) ||
        !(feedforward_ramp_pps <=
            WHEEL_SPEED_CONTROL_TARGET_MAX_PPS)) {
        g_snapshot.last_result =
            WHEEL_SPEED_CONTROL_BAD_FEEDFORWARD_CONFIG;
        return g_snapshot.last_result;
    }
    if (g_snapshot.running) {
        if ((g_owner_mode == owner_mode) &&
            (ControlSupervisor_GetMode() == owner_mode) &&
            (g_feedforward_boost_permille ==
                feedforward_boost_permille) &&
            (g_feedforward_ramp_pps == feedforward_ramp_pps)) {
            return WHEEL_SPEED_CONTROL_OK;
        }
        g_snapshot.last_result = WHEEL_SPEED_CONTROL_BUSY;
        return g_snapshot.last_result;
    }
    if (ControlSupervisor_BeginClosedLoop(owner_mode, now_ms) !=
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
    g_snapshot.command_age_ms = 0U;
    g_snapshot.last_result = WHEEL_SPEED_CONTROL_OK;
    g_snapshot.owner_mode = owner_mode;
    g_snapshot.running = true;
    g_owner_mode = owner_mode;
    g_feedforward_boost_permille = feedforward_boost_permille;
    g_feedforward_ramp_pps = feedforward_ramp_pps;
    g_left_feedforward_startup_active =
        (feedforward_ramp_pps > 0.0f) ||
        (feedforward_boost_permille > 0U);
    g_right_feedforward_startup_active =
        g_left_feedforward_startup_active;
    g_left_requested_pps = 0.0f;
    g_right_requested_pps = 0.0f;
    g_last_update_ms = now_ms;
    g_last_command_ms = now_ms;
    g_command_deadline_ms = now_ms +
        WHEEL_SPEED_CONTROL_COMMAND_LEASE_MS;
    return WHEEL_SPEED_CONTROL_OK;
}

wheel_speed_control_result_t WheelSpeedControl_SetTargets(
    float left_pps, float right_pps, uint32_t now_ms)
{
    if (!g_snapshot.running) {
        g_snapshot.last_result = WHEEL_SPEED_CONTROL_NOT_RUNNING;
        return g_snapshot.last_result;
    }
    if (ControlSupervisor_GetMode() != g_owner_mode) {
        wheel_speed_control_fault(
            WHEEL_SPEED_CONTROL_SUPERVISOR_BLOCKED,
            CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        return WHEEL_SPEED_CONTROL_SUPERVISOR_BLOCKED;
    }
    if (!wheel_speed_control_target_is_valid(left_pps) ||
        !wheel_speed_control_target_is_valid(right_pps)) {
        wheel_speed_control_fault(WHEEL_SPEED_CONTROL_BAD_TARGET,
            CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        return WHEEL_SPEED_CONTROL_BAD_TARGET;
    }

    g_left_requested_pps = left_pps;
    g_right_requested_pps = right_pps;
    g_last_command_ms = now_ms;
    g_command_deadline_ms = now_ms +
        WHEEL_SPEED_CONTROL_COMMAND_LEASE_MS;
    g_snapshot.command_age_ms = 0U;
    if ((left_pps == 0.0f) && (right_pps == 0.0f)) {
        g_snapshot.left_target_pps = 0.0f;
        g_snapshot.right_target_pps = 0.0f;
        wheel_speed_control_reset_pid_state();
        g_snapshot.left_output_permille = 0;
        g_snapshot.right_output_permille = 0;
        if (BoardWheelDrive_SetCommands(0, 0) !=
            BOARD_WHEEL_DRIVE_OK) {
            wheel_speed_control_fault(
                WHEEL_SPEED_CONTROL_OUTPUT_ERROR,
                CAR_CONTROL_BLOCK_EMERGENCY_STOP);
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
    g_snapshot.command_age_ms = now_ms - g_last_command_ms;
    if (wheel_speed_control_deadline_reached(
            now_ms, g_command_deadline_ms)) {
        wheel_speed_control_fault(
            WHEEL_SPEED_CONTROL_COMMAND_TIMEOUT,
            CAR_CONTROL_BLOCK_COMMAND_TIMEOUT);
        return;
    }
    if (ControlSupervisor_GetMode() != g_owner_mode) {
        wheel_speed_control_fault(
            WHEEL_SPEED_CONTROL_SUPERVISOR_BLOCKED,
            CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        return;
    }

    elapsed_ms = now_ms - g_last_update_ms;
    if (elapsed_ms < ENCODER_INPUT_SAMPLE_INTERVAL_MS) {
        return;
    }
    if (!EncoderInput_GetSnapshot(ENCODER_INPUT_0, &left_encoder) ||
        !EncoderInput_GetSnapshot(ENCODER_INPUT_1, &right_encoder)) {
        wheel_speed_control_fault(WHEEL_SPEED_CONTROL_ENCODER_ERROR,
            CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        return;
    }

    dt_s = (float) elapsed_ms / 1000.0f;
    g_last_update_ms = now_ms;
    target_max_delta = wheel_speed_control_target_slew_rate() * dt_s;
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
        g_left_output_limit, g_feedforward_boost_permille,
        g_feedforward_ramp_pps,
        &g_left_feedforward_startup_active);
    right_output = wheel_speed_control_update_one(&g_right_pid,
        g_snapshot.right_target_pps,
        (float) g_snapshot.right_measured_pps, dt_s,
        g_right_output_limit, g_feedforward_boost_permille,
        g_feedforward_ramp_pps,
        &g_right_feedforward_startup_active);
    g_snapshot.left_output_permille =
        wheel_speed_control_round_output(left_output);
    g_snapshot.right_output_permille =
        wheel_speed_control_round_output(right_output);

    if (BoardWheelDrive_SetCommands(
            g_snapshot.left_output_permille,
            g_snapshot.right_output_permille) != BOARD_WHEEL_DRIVE_OK) {
        wheel_speed_control_fault(WHEEL_SPEED_CONTROL_OUTPUT_ERROR,
            CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        return;
    }
    if (ControlSupervisor_RefreshClosedLoop(g_owner_mode, now_ms) !=
        CAR_CONTROL_REQUEST_OK) {
        wheel_speed_control_fault(
            WHEEL_SPEED_CONTROL_SUPERVISOR_BLOCKED,
            CAR_CONTROL_BLOCK_EMERGENCY_STOP);
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
    uint16_t output_limit, uint16_t feedforward_boost_permille,
    float feedforward_ramp_pps, bool *feedforward_startup_active)
{
    float feedforward;
    float correction;
    float target_magnitude;
    float measured_magnitude;
    float active_boost = 0.0f;
    float offset_scale = 1.0f;

    if (target_pps == 0.0f) {
        PID_Reset(pid);
        return 0.0f;
    }

    target_magnitude = (target_pps > 0.0f) ? target_pps : -target_pps;
    measured_magnitude = (measured_pps > 0.0f) ?
        measured_pps : -measured_pps;
    if ((feedforward_ramp_pps > 0.0f) &&
        (target_magnitude < feedforward_ramp_pps)) {
        offset_scale = target_magnitude / feedforward_ramp_pps;
    }
    if ((feedforward_startup_active != NULL) &&
        !*feedforward_startup_active &&
        (feedforward_ramp_pps > 0.0f) &&
        (target_magnitude < feedforward_ramp_pps) &&
        (measured_magnitude <= WHEEL_SPEED_BOOST_MEASURED_MAX_PPS)) {
        *feedforward_startup_active = true;
    }
    if ((feedforward_startup_active != NULL) &&
        *feedforward_startup_active) {
        if ((measured_magnitude >
                WHEEL_SPEED_BOOST_MEASURED_MAX_PPS) &&
            ((feedforward_ramp_pps <= 0.0f) ||
                (target_magnitude >= feedforward_ramp_pps))) {
            *feedforward_startup_active = false;
        } else {
            offset_scale = 1.0f;
            active_boost = (float) feedforward_boost_permille;
        }
    }
    feedforward = WHEEL_SPEED_FEEDFORWARD_OFFSET * offset_scale +
        WHEEL_SPEED_FEEDFORWARD_GAIN * target_magnitude +
        active_boost;
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
    if (g_owner_mode == CAR_CONTROL_MODE_YAW) {
        if (((current > 0.0f) && (requested < 0.0f)) ||
            ((current < 0.0f) && (requested > 0.0f))) {
            return 0.0f;
        }
        if (((current > 0.0f) && (requested >= 0.0f) &&
                (requested < current)) ||
            ((current < 0.0f) && (requested <= 0.0f) &&
                (requested > current))) {
            return requested;
        }
    }
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

static float wheel_speed_control_target_slew_rate(void)
{
    if (g_owner_mode == CAR_CONTROL_MODE_POSITION) {
        return WHEEL_SPEED_POSITION_TARGET_SLEW_PPS_PER_S;
    }
    if (g_owner_mode == CAR_CONTROL_MODE_YAW) {
        return WHEEL_SPEED_YAW_TARGET_SLEW_PPS_PER_S;
    }
    if (g_owner_mode == CAR_CONTROL_MODE_HEADING) {
        return WHEEL_SPEED_HEADING_TARGET_SLEW_PPS_PER_S;
    }
    return WHEEL_SPEED_TARGET_SLEW_PPS_PER_S;
}

static bool wheel_speed_control_deadline_reached(
    uint32_t now_ms, uint32_t deadline_ms)
{
    return ((int32_t) (now_ms - deadline_ms) >= 0);
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
    g_feedforward_boost_permille = 0U;
    g_feedforward_ramp_pps = 0.0f;
    g_left_feedforward_startup_active = false;
    g_right_feedforward_startup_active = false;
    g_command_deadline_ms = 0U;
    wheel_speed_control_reset_pid_state();
    BoardWheelDrive_SetZero();
}

static void wheel_speed_control_fault(wheel_speed_control_result_t result,
    car_control_block_reason_t reason)
{
    wheel_speed_control_deactivate();
    g_snapshot.last_result = result;
    ControlSupervisor_EmergencyStop(reason);
}
