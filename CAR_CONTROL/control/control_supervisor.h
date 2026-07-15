#ifndef CONTROL_CONTROL_SUPERVISOR_H
#define CONTROL_CONTROL_SUPERVISOR_H

#include <stdbool.h>
#include <stdint.h>

#define CONTROL_SUPERVISOR_OPEN_LOOP_MAX_MS 3500U
#define CONTROL_SUPERVISOR_SPEED_LEASE_MS     200U

typedef enum {
    CAR_CONTROL_MODE_SAFE_IDLE = 0,
    CAR_CONTROL_MODE_OPEN_LOOP,
    CAR_CONTROL_MODE_SPEED,
    CAR_CONTROL_MODE_POSITION,
    CAR_CONTROL_MODE_YAW,
    CAR_CONTROL_MODE_LINE_TRACKING
} car_control_mode_t;

typedef enum {
    CAR_CONTROL_MOTOR_A = 0,
    CAR_CONTROL_MOTOR_B,
    CAR_CONTROL_MOTOR_BOTH,
    CAR_CONTROL_MOTOR_COUNT
} car_control_motor_t;

typedef enum {
    CAR_CONTROL_BLOCK_NONE = 0,
    CAR_CONTROL_BLOCK_STARTUP,
    CAR_CONTROL_BLOCK_HARDWARE_UNVERIFIED,
    CAR_CONTROL_BLOCK_SUSPICIOUS_RESET,
    CAR_CONTROL_BLOCK_EMERGENCY_STOP,
    CAR_CONTROL_BLOCK_TEST_COMPLETE,
    CAR_CONTROL_BLOCK_OPERATOR_STOP
} car_control_block_reason_t;

typedef enum {
    CAR_CONTROL_REQUEST_OK = 0,
    CAR_CONTROL_REQUEST_BLOCKED
} car_control_request_result_t;

void ControlSupervisor_Init(bool suspicious_reset);
void ControlSupervisor_Task(uint32_t now_ms);
void ControlSupervisor_EmergencyStop(car_control_block_reason_t reason);
car_control_request_result_t ControlSupervisor_BeginOpenLoopTest(
    car_control_motor_t motor, uint32_t now_ms, uint32_t duration_ms);
car_control_request_result_t ControlSupervisor_BeginSpeedControl(
    uint32_t now_ms);
car_control_request_result_t ControlSupervisor_RefreshSpeedControl(
    uint32_t now_ms);

car_control_request_result_t ControlSupervisor_RequestMode(
    car_control_mode_t mode);
car_control_mode_t ControlSupervisor_GetMode(void);
car_control_block_reason_t ControlSupervisor_GetBlockReason(void);
const char *ControlSupervisor_GetModeText(void);
const char *ControlSupervisor_GetBlockReasonText(void);

#endif
