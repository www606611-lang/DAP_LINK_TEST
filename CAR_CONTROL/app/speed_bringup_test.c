#include "speed_bringup_test.h"

#include "encoder_input.h"
#include "wheel_speed_control.h"

#include <stddef.h>

#define SPEED_BRINGUP_RAMP_MS           1000U
#define SPEED_BRINGUP_RUN_MS            5000U
#define SPEED_BRINGUP_DEFAULT_TARGET_PPS   3500.0f
#define SPEED_BRINGUP_DEFAULT_OUTPUT_LIMIT  700U

static speed_bringup_test_state_t g_state;
static speed_bringup_config_t g_config;
static uint32_t g_run_started_ms;
static uint32_t g_run_count;
static bool g_start_requested;
static bool g_stop_requested;

static void speed_bringup_start(uint32_t now_ms);
static void speed_bringup_stop(car_control_block_reason_t reason,
    speed_bringup_test_state_t next_state);

void SpeedBringupTest_Init(bool reset_locked)
{
    g_state = reset_locked ?
        SPEED_BRINGUP_TEST_LOCKED : SPEED_BRINGUP_TEST_READY;
    g_run_started_ms = 0U;
    g_run_count = 0U;
    g_start_requested = false;
    g_stop_requested = false;
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
            if (press_event || start_event) {
                speed_bringup_start(now_ms);
            }
            return;

        case SPEED_BRINGUP_TEST_RUNNING:
            if (press_event) {
                speed_bringup_stop(CAR_CONTROL_BLOCK_OPERATOR_STOP,
                    SPEED_BRINGUP_TEST_ABORTED);
                return;
            }

            if (!WheelSpeedControl_GetSnapshot(&snapshot) ||
                !snapshot.running ||
                (ControlSupervisor_GetMode() != CAR_CONTROL_MODE_SPEED)) {
                g_state = SPEED_BRINGUP_TEST_ABORTED;
                return;
            }

            elapsed_ms = now_ms - g_run_started_ms;
            if (elapsed_ms >= SPEED_BRINGUP_RUN_MS) {
                speed_bringup_stop(CAR_CONTROL_BLOCK_TEST_COMPLETE,
                    SPEED_BRINGUP_TEST_COMPLETE);
                return;
            }

            if (elapsed_ms < SPEED_BRINGUP_RAMP_MS) {
                target_pps = (g_config.target_pps *
                    (float) elapsed_ms) /
                    (float) SPEED_BRINGUP_RAMP_MS;
            } else {
                target_pps = g_config.target_pps;
            }

            if (WheelSpeedControl_SetTargets(target_pps, target_pps) !=
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
    if ((g_state == SPEED_BRINGUP_TEST_LOCKED) ||
        (g_state == SPEED_BRINGUP_TEST_RUNNING)) {
        return false;
    }
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

uint32_t SpeedBringupTest_GetRunCount(void)
{
    return g_run_count;
}

static void speed_bringup_start(uint32_t now_ms)
{
    if (WheelSpeedControl_SetOutputLimits(
            g_config.output_limit_permille,
            g_config.output_limit_permille) != WHEEL_SPEED_CONTROL_OK) {
        g_state = SPEED_BRINGUP_TEST_ABORTED;
        return;
    }

    EncoderInput_ResetAll();
    if (WheelSpeedControl_Start(now_ms) != WHEEL_SPEED_CONTROL_OK) {
        g_state = SPEED_BRINGUP_TEST_ABORTED;
        return;
    }
    if (WheelSpeedControl_SetTargets(0.0f, 0.0f) !=
        WHEEL_SPEED_CONTROL_OK) {
        g_state = SPEED_BRINGUP_TEST_ABORTED;
        return;
    }

    g_run_started_ms = now_ms;
    g_run_count++;
    g_state = SPEED_BRINGUP_TEST_RUNNING;
}

static void speed_bringup_stop(car_control_block_reason_t reason,
    speed_bringup_test_state_t next_state)
{
    WheelSpeedControl_Stop(reason);
    g_state = next_state;
}
