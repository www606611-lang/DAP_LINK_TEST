#include "position_bringup_test.h"

#include "control_supervisor.h"
#include "wheel_speed_control.h"

#include <stddef.h>

#define POSITION_BRINGUP_DEFAULT_TARGET_COUNTS 1060
#define POSITION_BRINGUP_DEFAULT_OUTPUT_LIMIT   650U
#define POSITION_BRINGUP_DEFAULT_TIMEOUT_MS     8000U
#define POSITION_BRINGUP_STRESS_PAUSE_MS         100U

static position_bringup_test_state_t g_state;
static position_bringup_config_t g_config;
static uint32_t g_run_count;
static bool g_start_requested;
static bool g_move_requested;
static bool g_stop_requested;
static int32_t g_requested_move_counts;
static bool g_waiting_next_move;
static position_bringup_profile_t g_requested_profile;
static position_bringup_profile_t g_active_profile;
static uint8_t g_stress_step_index;
static uint8_t g_completed_move_count;
static uint32_t g_worst_final_error_count;
static uint32_t g_left_recovery_count;
static uint32_t g_right_recovery_count;
static uint32_t g_next_move_ms;

static void position_bringup_start(
    uint32_t now_ms, position_bringup_profile_t profile,
    bool use_move_override, int32_t move_override_counts);
static bool position_bringup_start_move(
    uint32_t now_ms, int32_t delta_counts);
static void position_bringup_stop(car_control_block_reason_t reason,
    position_bringup_test_state_t next_state);
static bool position_bringup_profile_is_valid(
    position_bringup_profile_t profile);
static int32_t position_bringup_stress_delta(uint8_t step_index);
static uint32_t position_bringup_abs_i32(int32_t value);
static uint32_t position_bringup_max_u32(uint32_t left, uint32_t right);
static bool position_bringup_deadline_reached(
    uint32_t now_ms, uint32_t deadline_ms);

void PositionBringupTest_Init(bool reset_locked)
{
    g_state = reset_locked ?
        POSITION_BRINGUP_TEST_LOCKED : POSITION_BRINGUP_TEST_READY;
    g_run_count = 0U;
    g_start_requested = false;
    g_move_requested = false;
    g_stop_requested = false;
    g_requested_move_counts = 0;
    g_waiting_next_move = false;
    g_requested_profile = POSITION_BRINGUP_PROFILE_SINGLE;
    g_active_profile = POSITION_BRINGUP_PROFILE_SINGLE;
    g_stress_step_index = 0U;
    g_completed_move_count = 0U;
    g_worst_final_error_count = 0U;
    g_left_recovery_count = 0U;
    g_right_recovery_count = 0U;
    g_next_move_ms = 0U;
    (void) WheelPositionControl_GetConfig(&g_config.control);
    g_config.target_counts = POSITION_BRINGUP_DEFAULT_TARGET_COUNTS;
    g_config.output_limit_permille =
        POSITION_BRINGUP_DEFAULT_OUTPUT_LIMIT;
    g_config.timeout_ms = POSITION_BRINGUP_DEFAULT_TIMEOUT_MS;
}

void PositionBringupTest_Task(uint32_t now_ms, bool press_event)
{
    wheel_position_control_snapshot_t position;
    bool start_event = g_start_requested;
    bool move_event = g_move_requested;
    bool stop_event = g_stop_requested;
    int32_t requested_move_counts = g_requested_move_counts;
    position_bringup_profile_t requested_profile = g_requested_profile;

    g_start_requested = false;
    g_move_requested = false;
    g_stop_requested = false;

    if (stop_event) {
        if (g_state == POSITION_BRINGUP_TEST_RUNNING) {
            position_bringup_stop(CAR_CONTROL_BLOCK_OPERATOR_STOP,
                POSITION_BRINGUP_TEST_ABORTED);
        }
        return;
    }

    switch (g_state) {
        case POSITION_BRINGUP_TEST_LOCKED:
            return;

        case POSITION_BRINGUP_TEST_READY:
        case POSITION_BRINGUP_TEST_COMPLETE:
        case POSITION_BRINGUP_TEST_ABORTED:
            if (press_event || start_event || move_event) {
                position_bringup_start(now_ms,
                    (press_event || move_event) ?
                        POSITION_BRINGUP_PROFILE_SINGLE : requested_profile,
                    move_event, requested_move_counts);
            }
            return;

        case POSITION_BRINGUP_TEST_RUNNING:
            if (press_event) {
                position_bringup_stop(CAR_CONTROL_BLOCK_OPERATOR_STOP,
                    POSITION_BRINGUP_TEST_ABORTED);
                return;
            }
            if (g_waiting_next_move) {
                if (position_bringup_deadline_reached(
                        now_ms, g_next_move_ms)) {
                    g_stress_step_index++;
                    g_waiting_next_move = false;
                    if (!position_bringup_start_move(now_ms,
                            position_bringup_stress_delta(
                                g_stress_step_index))) {
                        g_state = POSITION_BRINGUP_TEST_ABORTED;
                    }
                }
                return;
            }
            if (!WheelPositionControl_GetSnapshot(&position)) {
                g_state = POSITION_BRINGUP_TEST_ABORTED;
                return;
            }
            if (!position.running) {
                if (position.settled &&
                    (position.last_result == WHEEL_POSITION_CONTROL_OK) &&
                    (ControlSupervisor_GetBlockReason() ==
                        CAR_CONTROL_BLOCK_TEST_COMPLETE)) {
                    uint32_t final_error = position_bringup_max_u32(
                        position_bringup_abs_i32(
                            position.left_error_count),
                        position_bringup_abs_i32(
                            position.right_error_count));

                    if (final_error > g_worst_final_error_count) {
                        g_worst_final_error_count = final_error;
                    }
                    g_left_recovery_count +=
                        position.left_recovery_count;
                    g_right_recovery_count +=
                        position.right_recovery_count;
                    g_completed_move_count++;
                    if ((g_active_profile ==
                            POSITION_BRINGUP_PROFILE_STRESS) &&
                        (g_completed_move_count <
                            POSITION_BRINGUP_STRESS_STEP_COUNT)) {
                        g_waiting_next_move = true;
                        g_next_move_ms = now_ms +
                            POSITION_BRINGUP_STRESS_PAUSE_MS;
                    } else {
                        g_state = POSITION_BRINGUP_TEST_COMPLETE;
                    }
                } else {
                    g_state = POSITION_BRINGUP_TEST_ABORTED;
                }
            }
            return;

        default:
            position_bringup_stop(CAR_CONTROL_BLOCK_EMERGENCY_STOP,
                POSITION_BRINGUP_TEST_ABORTED);
            return;
    }
}

position_bringup_config_result_t PositionBringupTest_SetConfig(
    const position_bringup_config_t *config)
{
    if (config == NULL) {
        return POSITION_BRINGUP_CONFIG_BAD_ARGUMENT;
    }
    if (g_state == POSITION_BRINGUP_TEST_RUNNING) {
        return POSITION_BRINGUP_CONFIG_BUSY;
    }
    if (!WheelPositionControl_ConfigIsValid(&config->control) ||
        (config->target_counts == 0) ||
        (config->target_counts <
            -POSITION_BRINGUP_TARGET_MAX_COUNTS) ||
        (config->target_counts >
            POSITION_BRINGUP_TARGET_MAX_COUNTS) ||
        (config->output_limit_permille < POSITION_BRINGUP_OUTPUT_MIN) ||
        (config->output_limit_permille > POSITION_BRINGUP_OUTPUT_MAX) ||
        (config->timeout_ms < WHEEL_POSITION_CONTROL_RUN_MIN_MS) ||
        (config->timeout_ms > WHEEL_POSITION_CONTROL_RUN_MAX_MS)) {
        return POSITION_BRINGUP_CONFIG_BAD_RANGE;
    }
    if (WheelPositionControl_SetConfig(&config->control) !=
        WHEEL_POSITION_CONTROL_OK) {
        return POSITION_BRINGUP_CONFIG_BUSY;
    }

    g_config = *config;
    return POSITION_BRINGUP_CONFIG_OK;
}

bool PositionBringupTest_GetConfig(position_bringup_config_t *config)
{
    if (config == NULL) {
        return false;
    }
    *config = g_config;
    return true;
}

bool PositionBringupTest_RequestStart(void)
{
    return PositionBringupTest_RequestProfile(
        POSITION_BRINGUP_PROFILE_SINGLE);
}

bool PositionBringupTest_RequestProfile(position_bringup_profile_t profile)
{
    if ((g_state == POSITION_BRINGUP_TEST_LOCKED) ||
        (g_state == POSITION_BRINGUP_TEST_RUNNING) ||
        !position_bringup_profile_is_valid(profile) ||
        ((profile == POSITION_BRINGUP_PROFILE_STRESS) &&
            (position_bringup_abs_i32(g_config.target_counts) < 4U))) {
        return false;
    }
    g_requested_profile = profile;
    g_start_requested = true;
    g_move_requested = false;
    return true;
}

bool PositionBringupTest_RequestMove(int32_t delta_counts)
{
    if ((g_state == POSITION_BRINGUP_TEST_LOCKED) ||
        (g_state == POSITION_BRINGUP_TEST_RUNNING) ||
        (delta_counts == 0) ||
        (delta_counts < -POSITION_BRINGUP_TARGET_MAX_COUNTS) ||
        (delta_counts > POSITION_BRINGUP_TARGET_MAX_COUNTS)) {
        return false;
    }
    g_requested_move_counts = delta_counts;
    g_requested_profile = POSITION_BRINGUP_PROFILE_SINGLE;
    g_start_requested = false;
    g_move_requested = true;
    return true;
}

void PositionBringupTest_RequestStop(void)
{
    g_stop_requested = true;
}

position_bringup_test_state_t PositionBringupTest_GetState(void)
{
    return g_state;
}

const char *PositionBringupTest_GetStateText(void)
{
    switch (g_state) {
        case POSITION_BRINGUP_TEST_LOCKED:
            return "LOCKED";
        case POSITION_BRINGUP_TEST_READY:
            return "READY";
        case POSITION_BRINGUP_TEST_RUNNING:
            return "RUN";
        case POSITION_BRINGUP_TEST_COMPLETE:
            return "DONE";
        case POSITION_BRINGUP_TEST_ABORTED:
            return "ABORT";
        default:
            return "UNKNOWN";
    }
}

position_bringup_profile_t PositionBringupTest_GetProfile(void)
{
    return g_active_profile;
}

const char *PositionBringupTest_GetProfileText(void)
{
    switch (g_active_profile) {
        case POSITION_BRINGUP_PROFILE_SINGLE:
            return "SINGLE";
        case POSITION_BRINGUP_PROFILE_STRESS:
            return "STRESS";
        default:
            return "UNKNOWN";
    }
}

uint8_t PositionBringupTest_GetCurrentStep(void)
{
    return (g_active_profile == POSITION_BRINGUP_PROFILE_STRESS) ?
        (uint8_t) (g_stress_step_index + 1U) : 1U;
}

uint8_t PositionBringupTest_GetStepCount(void)
{
    return (g_active_profile == POSITION_BRINGUP_PROFILE_STRESS) ?
        POSITION_BRINGUP_STRESS_STEP_COUNT : 1U;
}

uint8_t PositionBringupTest_GetCompletedMoveCount(void)
{
    return g_completed_move_count;
}

uint32_t PositionBringupTest_GetWorstFinalErrorCount(void)
{
    return g_worst_final_error_count;
}

uint32_t PositionBringupTest_GetLeftRecoveryCount(void)
{
    return g_left_recovery_count;
}

uint32_t PositionBringupTest_GetRightRecoveryCount(void)
{
    return g_right_recovery_count;
}

uint32_t PositionBringupTest_GetRunCount(void)
{
    return g_run_count;
}

static void position_bringup_start(
    uint32_t now_ms, position_bringup_profile_t profile,
    bool use_move_override, int32_t move_override_counts)
{
    int32_t first_delta;

    if (!position_bringup_profile_is_valid(profile)) {
        g_state = POSITION_BRINGUP_TEST_ABORTED;
        return;
    }
    if (WheelPositionControl_SetConfig(&g_config.control) !=
        WHEEL_POSITION_CONTROL_OK) {
        g_state = POSITION_BRINGUP_TEST_ABORTED;
        return;
    }
    if (WheelSpeedControl_SetOutputLimits(
            g_config.output_limit_permille,
            g_config.output_limit_permille) != WHEEL_SPEED_CONTROL_OK) {
        g_state = POSITION_BRINGUP_TEST_ABORTED;
        return;
    }

    g_active_profile = profile;
    g_stress_step_index = 0U;
    g_completed_move_count = 0U;
    g_worst_final_error_count = 0U;
    g_left_recovery_count = 0U;
    g_right_recovery_count = 0U;
    g_waiting_next_move = false;
    first_delta = use_move_override ? move_override_counts :
        ((profile == POSITION_BRINGUP_PROFILE_STRESS) ?
            position_bringup_stress_delta(0U) : g_config.target_counts);
    if (!position_bringup_start_move(now_ms, first_delta)) {
        g_state = POSITION_BRINGUP_TEST_ABORTED;
        return;
    }

    g_run_count++;
    g_state = POSITION_BRINGUP_TEST_RUNNING;
}

static bool position_bringup_start_move(
    uint32_t now_ms, int32_t delta_counts)
{
    return WheelPositionControl_StartRelative(
        delta_counts, delta_counts,
        g_config.timeout_ms, now_ms) == WHEEL_POSITION_CONTROL_OK;
}

static void position_bringup_stop(car_control_block_reason_t reason,
    position_bringup_test_state_t next_state)
{
    WheelPositionControl_Stop(reason);
    g_waiting_next_move = false;
    g_state = next_state;
}

static bool position_bringup_profile_is_valid(
    position_bringup_profile_t profile)
{
    return profile <= POSITION_BRINGUP_PROFILE_STRESS;
}

static int32_t position_bringup_stress_delta(uint8_t step_index)
{
    int32_t magnitude = (g_config.target_counts < 0) ?
        -g_config.target_counts : g_config.target_counts;

    switch ((uint8_t) (step_index % 8U)) {
        case 0U:
            return magnitude;
        case 1U:
            return -magnitude;
        case 2U:
            return magnitude / 2;
        case 3U:
            return -(magnitude / 2);
        case 4U:
            return magnitude * 2;
        case 5U:
            return -(magnitude * 2);
        case 6U:
            return magnitude / 4;
        case 7U:
        default:
            return -(magnitude / 4);
    }
}

static uint32_t position_bringup_abs_i32(int32_t value)
{
    if (value < 0) {
        return (uint32_t) (-(value + 1)) + 1U;
    }
    return (uint32_t) value;
}

static uint32_t position_bringup_max_u32(uint32_t left, uint32_t right)
{
    return (left > right) ? left : right;
}

static bool position_bringup_deadline_reached(
    uint32_t now_ms, uint32_t deadline_ms)
{
    return ((int32_t) (now_ms - deadline_ms) >= 0);
}
