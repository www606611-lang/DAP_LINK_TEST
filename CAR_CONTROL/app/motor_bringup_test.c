#include "motor_bringup_test.h"

#include "at8236_motor.h"
#include "board_resources.h"
#include "control_supervisor.h"
#include "encoder_input.h"

#define MOTOR_BRINGUP_RAMP_MS          500U
#define MOTOR_BRINGUP_RUN_MS          3000U
#define MOTOR_BRINGUP_LEASE_MS        3200U
#define MOTOR_BRINGUP_TARGET_PERMILLE  700
#define MOTOR_BRINGUP_CONTROL_MOTOR    CAR_CONTROL_MOTOR_BOTH

static motor_bringup_test_state_t g_state;
static uint32_t g_run_started_ms;
static uint32_t g_run_count;
static int16_t g_command_a;
static int16_t g_command_b;

static void motor_bringup_start(uint32_t now_ms);
static void motor_bringup_stop(car_control_block_reason_t reason,
    motor_bringup_test_state_t next_state);
static int16_t motor_bringup_get_ramp_command(uint32_t elapsed_ms);

void MotorBringupTest_Init(bool reset_locked)
{
    g_state = reset_locked ?
        MOTOR_BRINGUP_TEST_LOCKED : MOTOR_BRINGUP_TEST_READY;
    g_run_started_ms = 0U;
    g_run_count = 0U;
    g_command_a = 0;
    g_command_b = 0;
    AT8236_MotorInit();
    AT8236_MotorSetInverted(AT8236_MOTOR_A,
        BOARD_MOTOR_A_FORWARD_INVERTED != 0);
    AT8236_MotorSetInverted(AT8236_MOTOR_B,
        BOARD_MOTOR_B_FORWARD_INVERTED != 0);
}

void MotorBringupTest_Task(uint32_t now_ms, bool press_event)
{
    uint32_t elapsed_ms;
    int16_t command_a;
    int16_t command_b;

    switch (g_state) {
        case MOTOR_BRINGUP_TEST_LOCKED:
            return;

        case MOTOR_BRINGUP_TEST_READY:
        case MOTOR_BRINGUP_TEST_COMPLETE:
        case MOTOR_BRINGUP_TEST_ABORTED:
            if (press_event) {
                motor_bringup_start(now_ms);
            }
            return;

        case MOTOR_BRINGUP_TEST_RUNNING:
            if (press_event) {
                motor_bringup_stop(CAR_CONTROL_BLOCK_OPERATOR_STOP,
                    MOTOR_BRINGUP_TEST_ABORTED);
                return;
            }

            if (ControlSupervisor_GetMode() !=
                CAR_CONTROL_MODE_OPEN_LOOP) {
                AT8236_MotorStopAll();
                g_command_a = 0;
                g_command_b = 0;
                g_state = MOTOR_BRINGUP_TEST_COMPLETE;
                return;
            }

            elapsed_ms = now_ms - g_run_started_ms;
            if (elapsed_ms >= MOTOR_BRINGUP_RUN_MS) {
                motor_bringup_stop(CAR_CONTROL_BLOCK_TEST_COMPLETE,
                    MOTOR_BRINGUP_TEST_COMPLETE);
                return;
            }

            command_a = motor_bringup_get_ramp_command(elapsed_ms);
            command_b = command_a;

            if (!AT8236_MotorSetCommand(
                    AT8236_MOTOR_A, command_a) ||
                !AT8236_MotorSetCommand(
                    AT8236_MOTOR_B, command_b)) {
                motor_bringup_stop(CAR_CONTROL_BLOCK_EMERGENCY_STOP,
                    MOTOR_BRINGUP_TEST_ABORTED);
                return;
            }
            g_command_a = command_a;
            g_command_b = command_b;
            return;

        default:
            motor_bringup_stop(CAR_CONTROL_BLOCK_EMERGENCY_STOP,
                MOTOR_BRINGUP_TEST_ABORTED);
            return;
    }
}

motor_bringup_test_state_t MotorBringupTest_GetState(void)
{
    return g_state;
}

const char *MotorBringupTest_GetStateText(void)
{
    switch (g_state) {
        case MOTOR_BRINGUP_TEST_LOCKED:
            return "LOCKED";
        case MOTOR_BRINGUP_TEST_READY:
            return "READY";
        case MOTOR_BRINGUP_TEST_RUNNING:
            return "RUN";
        case MOTOR_BRINGUP_TEST_COMPLETE:
            return "DONE";
        case MOTOR_BRINGUP_TEST_ABORTED:
            return "ABORT";
        default:
            return "UNKNOWN";
    }
}

int16_t MotorBringupTest_GetCommand(void)
{
    return g_command_a;
}

int16_t MotorBringupTest_GetCommandA(void)
{
    return g_command_a;
}

int16_t MotorBringupTest_GetCommandB(void)
{
    return g_command_b;
}

uint32_t MotorBringupTest_GetRunCount(void)
{
    return g_run_count;
}

uint32_t MotorBringupTest_GetMotorChannel(void)
{
    return (uint32_t) MOTOR_BRINGUP_CONTROL_MOTOR;
}

const char *MotorBringupTest_GetMotorText(void)
{
    return "AB";
}

static void motor_bringup_start(uint32_t now_ms)
{
    EncoderInput_ResetAll();
    if (ControlSupervisor_BeginOpenLoopTest(
            MOTOR_BRINGUP_CONTROL_MOTOR,
            now_ms, MOTOR_BRINGUP_LEASE_MS) !=
        CAR_CONTROL_REQUEST_OK) {
        g_state = MOTOR_BRINGUP_TEST_ABORTED;
        return;
    }

    g_run_started_ms = now_ms;
    g_run_count++;
    g_command_a = 0;
    g_command_b = 0;
    if (!AT8236_MotorSetCommand(AT8236_MOTOR_A, 0) ||
        !AT8236_MotorSetCommand(AT8236_MOTOR_B, 0)) {
        motor_bringup_stop(CAR_CONTROL_BLOCK_EMERGENCY_STOP,
            MOTOR_BRINGUP_TEST_ABORTED);
        return;
    }
    g_state = MOTOR_BRINGUP_TEST_RUNNING;
}

static void motor_bringup_stop(car_control_block_reason_t reason,
    motor_bringup_test_state_t next_state)
{
    AT8236_MotorStopAll();
    ControlSupervisor_EmergencyStop(reason);
    g_command_a = 0;
    g_command_b = 0;
    g_state = next_state;
}

static int16_t motor_bringup_get_ramp_command(uint32_t elapsed_ms)
{
    if (elapsed_ms >= MOTOR_BRINGUP_RAMP_MS) {
        return MOTOR_BRINGUP_TARGET_PERMILLE;
    }
    return (int16_t) ((elapsed_ms * MOTOR_BRINGUP_TARGET_PERMILLE) /
        MOTOR_BRINGUP_RAMP_MS);
}
