#include "speed_bringup_test.h"

#include "encoder_input.h"
#include "wheel_speed_control.h"

#include <stddef.h>

#define SPEED_BRINGUP_RAMP_MS             1000U
#define SPEED_BRINGUP_RUN_MS              5000U
#define SPEED_BRINGUP_STEP_RUN_MS         7000U
#define SPEED_BRINGUP_REVERSE_RUN_MS      9000U
#define SPEED_BRINGUP_SWEEP_RUN_MS       11000U
#define SPEED_BRINGUP_LEASE_RUN_MS         3000U
#define SPEED_BRINGUP_LEASE_STALE_MS       1500U
#define SPEED_BRINGUP_DEFAULT_TARGET_PPS   3500.0f
#define SPEED_BRINGUP_DEFAULT_OUTPUT_LIMIT  650U

static speed_bringup_test_state_t g_state;
static speed_bringup_config_t g_config;
static uint32_t g_run_started_ms;
static uint32_t g_run_count;
static bool g_start_requested;
static bool g_stop_requested;
static speed_bringup_profile_t g_requested_profile;
static speed_bringup_profile_t g_active_profile;

static void speed_bringup_start(
    uint32_t now_ms, speed_bringup_profile_t profile);
static void speed_bringup_stop(car_control_block_reason_t reason,
    speed_bringup_test_state_t next_state);
static bool speed_bringup_profile_is_valid(speed_bringup_profile_t profile);
static uint32_t speed_bringup_profile_duration_ms(
    speed_bringup_profile_t profile);
static float speed_bringup_profile_target(
    speed_bringup_profile_t profile, uint32_t elapsed_ms);
static car_control_mode_t speed_bringup_profile_owner(
    speed_bringup_profile_t profile);

void SpeedBringupTest_Init(bool reset_locked)
{
    g_state = reset_locked ?
        SPEED_BRINGUP_TEST_LOCKED : SPEED_BRINGUP_TEST_READY;
    g_run_started_ms = 0U;
    g_run_count = 0U;
    g_start_requested = false;
    g_stop_requested = false;
    g_requested_profile = SPEED_BRINGUP_PROFILE_RAMP;
    g_active_profile = SPEED_BRINGUP_PROFILE_RAMP;
    (void) WheelSpeedControl_GetTunings(&g_config.pid);
    g_config.target_pps = SPEED_BRINGUP_DEFAULT_TARGET_PPS;
    g_config.output_limit_permille =
        SPEED_BRINGUP_DEFAULT_OUTPUT_LIMIT;
}

void SpeedBringupTest_Task(uint32_t now_ms, bool press_event)
{
    uint32_t elapsed_ms;
    float target_pps;
    wheel_speed_control_snapshot_t snapshot;
    bool start_event = g_start_requested;
    bool stop_event = g_stop_requested;
    speed_bringup_profile_t requested_profile = g_requested_profile;

    g_start_requested = false;
    g_stop_requested = false;

    if (stop_event) {
        if (g_state == SPEED_BRINGUP_TEST_RUNNING) {
            speed_bringup_stop(CAR_CONTROL_BLOCK_OPERATOR_STOP,
                SPEED_BRINGUP_TEST_ABORTED);
        } else if (g_state != SPEED_BRINGUP_TEST_LOCKED) {
            WheelSpeedControl_Stop(CAR_CONTROL_BLOCK_OPERATOR_STOP);
            g_state = SPEED_BRINGUP_TEST_ABORTED;
        }
        return;
    }

    switch (g_state) {
        case SPEED_BRINGUP_TEST_LOCKED:
            return;

        case SPEED_BRINGUP_TEST_READY:
        case SPEED_BRINGUP_TEST_COMPLETE:
        case SPEED_BRINGUP_TEST_ABORTED:
            if (press_event) {
                speed_bringup_start(
                    now_ms, SPEED_BRINGUP_PROFILE_RAMP);
            } else if (start_event) {
                speed_bringup_start(now_ms, requested_profile);
            }
            return;

        case SPEED_BRINGUP_TEST_RUNNING:
            if (press_event) {
                speed_bringup_stop(CAR_CONTROL_BLOCK_OPERATOR_STOP,
                    SPEED_BRINGUP_TEST_ABORTED);
                return;
            }

            if (!WheelSpeedControl_GetSnapshot(&snapshot)) {
                g_state = SPEED_BRINGUP_TEST_ABORTED;
                return;
            }
            if (!snapshot.running) {
                if ((g_active_profile ==
                        SPEED_BRINGUP_PROFILE_LEASE_TEST) &&
                    (snapshot.last_result ==
                        WHEEL_SPEED_CONTROL_COMMAND_TIMEOUT) &&
                    (ControlSupervisor_GetBlockReason() ==
                        CAR_CONTROL_BLOCK_COMMAND_TIMEOUT)) {
                    g_state = SPEED_BRINGUP_TEST_COMPLETE;
                } else {
                    g_state = SPEED_BRINGUP_TEST_ABORTED;
                }
                return;
            }
            if ((snapshot.owner_mode !=
                    speed_bringup_profile_owner(g_active_profile)) ||
                (ControlSupervisor_GetMode() != snapshot.owner_mode)) {
                g_state = SPEED_BRINGUP_TEST_ABORTED;
                return;
            }

            elapsed_ms = now_ms - g_run_started_ms;
            if (elapsed_ms >=
                speed_bringup_profile_duration_ms(g_active_profile)) {
                speed_bringup_stop(CAR_CONTROL_BLOCK_TEST_COMPLETE,
                    SPEED_BRINGUP_TEST_COMPLETE);
                return;
            }

            target_pps = speed_bringup_profile_target(
                g_active_profile, elapsed_ms);

            if ((g_active_profile ==
                    SPEED_BRINGUP_PROFILE_LEASE_TEST) &&
                (elapsed_ms >= SPEED_BRINGUP_LEASE_STALE_MS)) {
                return;
            }

            if (WheelSpeedControl_SetTargets(
                    target_pps, target_pps, now_ms) !=
                WHEEL_SPEED_CONTROL_OK) {
                g_state = SPEED_BRINGUP_TEST_ABORTED;
            }
            return;

        default:
            speed_bringup_stop(CAR_CONTROL_BLOCK_EMERGENCY_STOP,
                SPEED_BRINGUP_TEST_ABORTED);
            return;
    }
}

speed_bringup_config_result_t SpeedBringupTest_SetConfig(
    const speed_bringup_config_t *config)
{
    if (config == NULL) {
        return SPEED_BRINGUP_CONFIG_BAD_ARGUMENT;
    }
    if (g_state == SPEED_BRINGUP_TEST_RUNNING) {
        return SPEED_BRINGUP_CONFIG_BUSY;
    }
    if (!WheelSpeedControl_TuningsAreValid(&config->pid) ||
        (config->target_pps < SPEED_BRINGUP_TARGET_MIN_PPS) ||
        (config->target_pps > SPEED_BRINGUP_TARGET_MAX_PPS) ||
        (config->output_limit_permille < SPEED_BRINGUP_OUTPUT_MIN) ||
        (config->output_limit_permille > SPEED_BRINGUP_OUTPUT_MAX)) {
        return SPEED_BRINGUP_CONFIG_BAD_RANGE;
    }
    if (WheelSpeedControl_SetTunings(&config->pid) !=
        WHEEL_SPEED_CONTROL_OK) {
        return SPEED_BRINGUP_CONFIG_BUSY;
    }

    g_config = *config;
    return SPEED_BRINGUP_CONFIG_OK;
}

bool SpeedBringupTest_GetConfig(speed_bringup_config_t *config)
{
    if (config == NULL) {
        return false;
    }
    *config = g_config;
    return true;
}

bool SpeedBringupTest_RequestStart(void)
{
    return SpeedBringupTest_RequestProfile(SPEED_BRINGUP_PROFILE_RAMP);
}

bool SpeedBringupTest_RequestProfile(speed_bringup_profile_t profile)
{
    if ((g_state == SPEED_BRINGUP_TEST_LOCKED) ||
        (g_state == SPEED_BRINGUP_TEST_RUNNING) ||
        !speed_bringup_profile_is_valid(profile)) {
        return false;
    }
    g_requested_profile = profile;
    g_active_profile = profile;
    g_start_requested = true;
    return true;
}

void SpeedBringupTest_RequestStop(void)
{
    g_stop_requested = true;
}

speed_bringup_test_state_t SpeedBringupTest_GetState(void)
{
    return g_state;
}

const char *SpeedBringupTest_GetStateText(void)
{
    switch (g_state) {
        case SPEED_BRINGUP_TEST_LOCKED:
            return "LOCKED";
        case SPEED_BRINGUP_TEST_READY:
            return "READY";
        case SPEED_BRINGUP_TEST_RUNNING:
            return "RUN";
        case SPEED_BRINGUP_TEST_COMPLETE:
            return "DONE";
        case SPEED_BRINGUP_TEST_ABORTED:
            return "ABORT";
        default:
            return "UNKNOWN";
    }
}

speed_bringup_profile_t SpeedBringupTest_GetProfile(void)
{
    return g_active_profile;
}

const char *SpeedBringupTest_GetProfileText(void)
{
    switch (g_active_profile) {
        case SPEED_BRINGUP_PROFILE_RAMP:
            return "RAMP";
        case SPEED_BRINGUP_PROFILE_STEP:
            return "STEP";
        case SPEED_BRINGUP_PROFILE_REVERSE:
            return "REV";
        case SPEED_BRINGUP_PROFILE_SWEEP:
            return "SWEEP";
        case SPEED_BRINGUP_PROFILE_LEASE_TEST:
            return "LEASE";
        default:
            return "UNKNOWN";
    }
}

uint32_t SpeedBringupTest_GetRunCount(void)
{
    return g_run_count;
}

static void speed_bringup_start(
    uint32_t now_ms, speed_bringup_profile_t profile)
{
    car_control_mode_t owner_mode;

    if (!speed_bringup_profile_is_valid(profile)) {
        g_state = SPEED_BRINGUP_TEST_ABORTED;
        return;
    }
    if (WheelSpeedControl_SetOutputLimits(
            g_config.output_limit_permille,
            g_config.output_limit_permille) != WHEEL_SPEED_CONTROL_OK) {
        g_state = SPEED_BRINGUP_TEST_ABORTED;
        return;
    }

    EncoderInput_ResetAll();
    owner_mode = speed_bringup_profile_owner(profile);
    if (WheelSpeedControl_StartForMode(owner_mode, now_ms) !=
        WHEEL_SPEED_CONTROL_OK) {
        g_state = SPEED_BRINGUP_TEST_ABORTED;
        return;
    }
    if (WheelSpeedControl_SetTargets(0.0f, 0.0f, now_ms) !=
        WHEEL_SPEED_CONTROL_OK) {
        g_state = SPEED_BRINGUP_TEST_ABORTED;
        return;
    }

    g_run_started_ms = now_ms;
    g_active_profile = profile;
    g_run_count++;
    g_state = SPEED_BRINGUP_TEST_RUNNING;
}

static void speed_bringup_stop(car_control_block_reason_t reason,
    speed_bringup_test_state_t next_state)
{
    WheelSpeedControl_Stop(reason);
    g_state = next_state;
}

static bool speed_bringup_profile_is_valid(speed_bringup_profile_t profile)
{
    return profile <= SPEED_BRINGUP_PROFILE_LEASE_TEST;
}

static uint32_t speed_bringup_profile_duration_ms(
    speed_bringup_profile_t profile)
{
    switch (profile) {
        case SPEED_BRINGUP_PROFILE_STEP:
            return SPEED_BRINGUP_STEP_RUN_MS;
        case SPEED_BRINGUP_PROFILE_REVERSE:
            return SPEED_BRINGUP_REVERSE_RUN_MS;
        case SPEED_BRINGUP_PROFILE_SWEEP:
            return SPEED_BRINGUP_SWEEP_RUN_MS;
        case SPEED_BRINGUP_PROFILE_LEASE_TEST:
            return SPEED_BRINGUP_LEASE_RUN_MS;
        case SPEED_BRINGUP_PROFILE_RAMP:
        default:
            return SPEED_BRINGUP_RUN_MS;
    }
}

static float speed_bringup_profile_target(
    speed_bringup_profile_t profile, uint32_t elapsed_ms)
{
    float target = g_config.target_pps;

    switch (profile) {
        case SPEED_BRINGUP_PROFILE_STEP:
            if (elapsed_ms < 1000U) {
                return 0.5f * target * (float) elapsed_ms / 1000.0f;
            }
            if (elapsed_ms < 2500U) {
                return 0.5f * target;
            }
            if (elapsed_ms < 4500U) {
                return target;
            }
            if (elapsed_ms < 6500U) {
                return 0.6f * target;
            }
            return 0.6f * target *
                (float) (SPEED_BRINGUP_STEP_RUN_MS - elapsed_ms) /
                500.0f;

        case SPEED_BRINGUP_PROFILE_REVERSE:
            target *= 0.7f;
            if (elapsed_ms < 1000U) {
                return target * (float) elapsed_ms / 1000.0f;
            }
            if (elapsed_ms < 3000U) {
                return target;
            }
            if (elapsed_ms < 4000U) {
                return target * (float) (4000U - elapsed_ms) / 1000.0f;
            }
            if (elapsed_ms < 4500U) {
                return 0.0f;
            }
            if (elapsed_ms < 5500U) {
                return -target * (float) (elapsed_ms - 4500U) / 1000.0f;
            }
            if (elapsed_ms < 7500U) {
                return -target;
            }
            if (elapsed_ms < 8500U) {
                return -target * (float) (8500U - elapsed_ms) / 1000.0f;
            }
            return 0.0f;

        case SPEED_BRINGUP_PROFILE_SWEEP:
            if (elapsed_ms < 1000U) {
                return 0.4f * target * (float) elapsed_ms / 1000.0f;
            }
            if (elapsed_ms < 3000U) {
                return 0.4f * target;
            }
            if (elapsed_ms < 5000U) {
                return 0.6f * target;
            }
            if (elapsed_ms < 7000U) {
                return 0.8f * target;
            }
            if (elapsed_ms < 10000U) {
                return target;
            }
            return target *
                (float) (SPEED_BRINGUP_SWEEP_RUN_MS - elapsed_ms) /
                1000.0f;

        case SPEED_BRINGUP_PROFILE_LEASE_TEST:
            target *= 0.5f;
            if (elapsed_ms < 1000U) {
                return target * (float) elapsed_ms / 1000.0f;
            }
            return target;

        case SPEED_BRINGUP_PROFILE_RAMP:
        default:
            if (elapsed_ms < SPEED_BRINGUP_RAMP_MS) {
                return target * (float) elapsed_ms /
                    (float) SPEED_BRINGUP_RAMP_MS;
            }
            return target;
    }
}

static car_control_mode_t speed_bringup_profile_owner(
    speed_bringup_profile_t profile)
{
    return (profile == SPEED_BRINGUP_PROFILE_LEASE_TEST) ?
        CAR_CONTROL_MODE_YAW : CAR_CONTROL_MODE_SPEED;
}
