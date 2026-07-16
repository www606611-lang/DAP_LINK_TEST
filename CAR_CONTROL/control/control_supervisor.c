#include "control_supervisor.h"

#include "board_motor_safe.h"
#include "board_wheel_drive.h"

static car_control_mode_t g_mode;
static car_control_block_reason_t g_block_reason;
static bool g_reset_locked;
static uint32_t g_mode_deadline_ms;

static bool control_supervisor_deadline_reached(
    uint32_t now_ms, uint32_t deadline_ms);

void ControlSupervisor_Init(bool suspicious_reset)
{
    BoardWheelDrive_SetZero();
    BoardMotorSafe_EmergencyStop();
    g_mode = CAR_CONTROL_MODE_SAFE_IDLE;
    g_reset_locked = suspicious_reset;
    g_mode_deadline_ms = 0U;
    g_block_reason = suspicious_reset ?
        CAR_CONTROL_BLOCK_SUSPICIOUS_RESET :
        CAR_CONTROL_BLOCK_HARDWARE_UNVERIFIED;
}

void ControlSupervisor_Task(uint32_t now_ms)
{
    bool closed_loop_active =
        ControlSupervisor_ModeCanOwnSpeedControl(g_mode);

    if (((g_mode == CAR_CONTROL_MODE_OPEN_LOOP) || closed_loop_active) &&
        control_supervisor_deadline_reached(
            now_ms, g_mode_deadline_ms)) {
        ControlSupervisor_EmergencyStop(
            (g_mode == CAR_CONTROL_MODE_OPEN_LOOP) ?
                CAR_CONTROL_BLOCK_TEST_COMPLETE :
                CAR_CONTROL_BLOCK_COMMAND_TIMEOUT);
        return;
    }

    if ((g_mode != CAR_CONTROL_MODE_SAFE_IDLE) &&
        (g_mode != CAR_CONTROL_MODE_OPEN_LOOP) &&
        !closed_loop_active) {
        ControlSupervisor_EmergencyStop(
            CAR_CONTROL_BLOCK_HARDWARE_UNVERIFIED);
    }
}

void ControlSupervisor_EmergencyStop(car_control_block_reason_t reason)
{
    BoardWheelDrive_SetZero();
    BoardMotorSafe_EmergencyStop();
    g_mode = CAR_CONTROL_MODE_SAFE_IDLE;
    g_block_reason = reason;
    g_mode_deadline_ms = 0U;
}

car_control_request_result_t ControlSupervisor_BeginOpenLoopTest(
    car_control_motor_t motor, uint32_t now_ms, uint32_t duration_ms)
{
    board_motor_channel_t board_channel;

    if (g_reset_locked ||
        ((uint32_t) motor >= (uint32_t) CAR_CONTROL_MOTOR_COUNT) ||
        (g_mode != CAR_CONTROL_MODE_SAFE_IDLE) ||
        (duration_ms == 0U) ||
        (duration_ms > CONTROL_SUPERVISOR_OPEN_LOOP_MAX_MS)) {
        return CAR_CONTROL_REQUEST_BLOCKED;
    }

    if (motor == CAR_CONTROL_MOTOR_A) {
        board_channel = BOARD_MOTOR_CHANNEL_A;
    } else if (motor == CAR_CONTROL_MOTOR_B) {
        board_channel = BOARD_MOTOR_CHANNEL_B;
    } else {
        board_channel = BOARD_MOTOR_CHANNEL_BOTH;
    }
    if (!BoardMotorSafe_Arm(board_channel)) {
        ControlSupervisor_EmergencyStop(CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        return CAR_CONTROL_REQUEST_BLOCKED;
    }
    BoardWheelDrive_SetZero();

    g_mode = CAR_CONTROL_MODE_OPEN_LOOP;
    g_block_reason = CAR_CONTROL_BLOCK_NONE;
    g_mode_deadline_ms = now_ms + duration_ms;
    return CAR_CONTROL_REQUEST_OK;
}

car_control_request_result_t ControlSupervisor_BeginSpeedControl(
    uint32_t now_ms)
{
    return ControlSupervisor_BeginClosedLoop(
        CAR_CONTROL_MODE_SPEED, now_ms);
}

car_control_request_result_t ControlSupervisor_RefreshSpeedControl(
    uint32_t now_ms)
{
    return ControlSupervisor_RefreshClosedLoop(
        CAR_CONTROL_MODE_SPEED, now_ms);
}

bool ControlSupervisor_ModeCanOwnSpeedControl(car_control_mode_t mode)
{
    return (mode == CAR_CONTROL_MODE_SPEED) ||
        (mode == CAR_CONTROL_MODE_POSITION) ||
        (mode == CAR_CONTROL_MODE_YAW) ||
        (mode == CAR_CONTROL_MODE_LINE_TRACKING);
}

car_control_request_result_t ControlSupervisor_BeginClosedLoop(
    car_control_mode_t owner_mode, uint32_t now_ms)
{
    if (g_reset_locked ||
        !ControlSupervisor_ModeCanOwnSpeedControl(owner_mode) ||
        (g_mode != CAR_CONTROL_MODE_SAFE_IDLE)) {
        return CAR_CONTROL_REQUEST_BLOCKED;
    }

    if (!BoardMotorSafe_Arm(BOARD_MOTOR_CHANNEL_BOTH)) {
        ControlSupervisor_EmergencyStop(CAR_CONTROL_BLOCK_EMERGENCY_STOP);
        return CAR_CONTROL_REQUEST_BLOCKED;
    }
    BoardWheelDrive_SetZero();

    g_mode = owner_mode;
    g_block_reason = CAR_CONTROL_BLOCK_NONE;
    g_mode_deadline_ms = now_ms +
        CONTROL_SUPERVISOR_CLOSED_LOOP_LEASE_MS;
    return CAR_CONTROL_REQUEST_OK;
}

car_control_request_result_t ControlSupervisor_RefreshClosedLoop(
    car_control_mode_t owner_mode, uint32_t now_ms)
{
    if (!ControlSupervisor_ModeCanOwnSpeedControl(owner_mode) ||
        (g_mode != owner_mode)) {
        return CAR_CONTROL_REQUEST_BLOCKED;
    }

    g_mode_deadline_ms = now_ms +
        CONTROL_SUPERVISOR_CLOSED_LOOP_LEASE_MS;
    return CAR_CONTROL_REQUEST_OK;
}

car_control_request_result_t ControlSupervisor_RequestMode(
    car_control_mode_t mode)
{
    if (mode == CAR_CONTROL_MODE_SAFE_IDLE) {
        ControlSupervisor_EmergencyStop(g_block_reason);
        return CAR_CONTROL_REQUEST_OK;
    }

    ControlSupervisor_EmergencyStop(
        CAR_CONTROL_BLOCK_HARDWARE_UNVERIFIED);
    return CAR_CONTROL_REQUEST_BLOCKED;
}

car_control_mode_t ControlSupervisor_GetMode(void)
{
    return g_mode;
}

car_control_block_reason_t ControlSupervisor_GetBlockReason(void)
{
    return g_block_reason;
}

const char *ControlSupervisor_GetModeText(void)
{
    switch (g_mode) {
        case CAR_CONTROL_MODE_SAFE_IDLE:
            return "SAFE_IDLE";
        case CAR_CONTROL_MODE_OPEN_LOOP:
            return "OPEN_LOOP";
        case CAR_CONTROL_MODE_SPEED:
            return "SPEED";
        case CAR_CONTROL_MODE_POSITION:
            return "POSITION";
        case CAR_CONTROL_MODE_YAW:
            return "YAW";
        case CAR_CONTROL_MODE_LINE_TRACKING:
            return "LINE";
        default:
            return "UNKNOWN";
    }
}

const char *ControlSupervisor_GetBlockReasonText(void)
{
    switch (g_block_reason) {
        case CAR_CONTROL_BLOCK_NONE:
            return "NONE";
        case CAR_CONTROL_BLOCK_STARTUP:
            return "STARTUP";
        case CAR_CONTROL_BLOCK_HARDWARE_UNVERIFIED:
            return "HW_UNVERIFIED";
        case CAR_CONTROL_BLOCK_SUSPICIOUS_RESET:
            return "RESET_LOCK";
        case CAR_CONTROL_BLOCK_EMERGENCY_STOP:
            return "EMERGENCY";
        case CAR_CONTROL_BLOCK_TEST_COMPLETE:
            return "TEST_DONE";
        case CAR_CONTROL_BLOCK_OPERATOR_STOP:
            return "USER_STOP";
        case CAR_CONTROL_BLOCK_COMMAND_TIMEOUT:
            return "CMD_TIMEOUT";
        default:
            return "UNKNOWN";
    }
}

static bool control_supervisor_deadline_reached(
    uint32_t now_ms, uint32_t deadline_ms)
{
    return ((int32_t) (now_ms - deadline_ms) >= 0);
}
