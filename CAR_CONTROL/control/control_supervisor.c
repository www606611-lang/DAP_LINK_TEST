#include "control_supervisor.h"

#include "board_motor_safe.h"

static car_control_mode_t g_mode;
static car_control_block_reason_t g_block_reason;

void ControlSupervisor_Init(bool suspicious_reset)
{
    BoardMotorSafe_EmergencyStop();
    g_mode = CAR_CONTROL_MODE_SAFE_IDLE;
    g_block_reason = suspicious_reset ?
        CAR_CONTROL_BLOCK_SUSPICIOUS_RESET :
        CAR_CONTROL_BLOCK_HARDWARE_UNVERIFIED;
}

void ControlSupervisor_Task(void)
{
    if (g_mode != CAR_CONTROL_MODE_SAFE_IDLE) {
        ControlSupervisor_EmergencyStop(
            CAR_CONTROL_BLOCK_HARDWARE_UNVERIFIED);
    }
}

void ControlSupervisor_EmergencyStop(car_control_block_reason_t reason)
{
    BoardMotorSafe_EmergencyStop();
    g_mode = CAR_CONTROL_MODE_SAFE_IDLE;
    g_block_reason = reason;
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
        case CAR_CONTROL_BLOCK_STARTUP:
            return "STARTUP";
        case CAR_CONTROL_BLOCK_HARDWARE_UNVERIFIED:
            return "HW_UNVERIFIED";
        case CAR_CONTROL_BLOCK_SUSPICIOUS_RESET:
            return "RESET_LOCK";
        case CAR_CONTROL_BLOCK_EMERGENCY_STOP:
            return "EMERGENCY";
        default:
            return "UNKNOWN";
    }
}
