#ifndef CONTROL_CONTROL_SUPERVISOR_H
#define CONTROL_CONTROL_SUPERVISOR_H

#include <stdbool.h>

typedef enum {
    CAR_CONTROL_MODE_SAFE_IDLE = 0,
    CAR_CONTROL_MODE_OPEN_LOOP,
    CAR_CONTROL_MODE_SPEED,
    CAR_CONTROL_MODE_POSITION,
    CAR_CONTROL_MODE_YAW,
    CAR_CONTROL_MODE_LINE_TRACKING
} car_control_mode_t;

typedef enum {
    CAR_CONTROL_BLOCK_STARTUP = 0,
    CAR_CONTROL_BLOCK_HARDWARE_UNVERIFIED,
    CAR_CONTROL_BLOCK_SUSPICIOUS_RESET,
    CAR_CONTROL_BLOCK_EMERGENCY_STOP
} car_control_block_reason_t;

typedef enum {
    CAR_CONTROL_REQUEST_OK = 0,
    CAR_CONTROL_REQUEST_BLOCKED
} car_control_request_result_t;

void ControlSupervisor_Init(bool suspicious_reset);
void ControlSupervisor_Task(void);
void ControlSupervisor_EmergencyStop(car_control_block_reason_t reason);

car_control_request_result_t ControlSupervisor_RequestMode(
    car_control_mode_t mode);
car_control_mode_t ControlSupervisor_GetMode(void);
car_control_block_reason_t ControlSupervisor_GetBlockReason(void);
const char *ControlSupervisor_GetModeText(void);
const char *ControlSupervisor_GetBlockReasonText(void);

#endif
