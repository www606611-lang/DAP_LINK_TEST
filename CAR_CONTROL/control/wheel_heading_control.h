#ifndef CONTROL_WHEEL_HEADING_CONTROL_H
#define CONTROL_WHEEL_HEADING_CONTROL_H

#include "control_supervisor.h"

#include <stdbool.h>
#include <stdint.h>

#define WHEEL_HEADING_CONTROL_UPDATE_INTERVAL_MS 10U
#define WHEEL_HEADING_CONTROL_COMMAND_LEASE_MS  100U
#define WHEEL_HEADING_CONTROL_TARGET_MAX_DEG    180.0f

typedef enum {
    WHEEL_HEADING_CONTROL_OK = 0,
    WHEEL_HEADING_CONTROL_BAD_CONFIG,
    WHEEL_HEADING_CONTROL_BAD_TARGET,
    WHEEL_HEADING_CONTROL_NOT_RUNNING,
    WHEEL_HEADING_CONTROL_BUSY,
    WHEEL_HEADING_CONTROL_IMU_ERROR,
    WHEEL_HEADING_CONTROL_SPEED_ERROR,
    WHEEL_HEADING_CONTROL_COMMAND_TIMEOUT,
    WHEEL_HEADING_CONTROL_STOPPED
} wheel_heading_control_result_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float max_correction_pps;
    float deadband_deg;
} wheel_heading_control_config_t;

typedef struct {
    float target_yaw_deg;
    float current_yaw_deg;
    float error_deg;
    float yaw_rate_dps;
    float base_speed_target_pps;
    float correction_target_pps;
    float left_speed_target_pps;
    float right_speed_target_pps;
    uint32_t update_count;
    uint32_t elapsed_ms;
    uint32_t command_age_ms;
    uint32_t last_interval_ms;
    uint32_t max_interval_ms;
    wheel_heading_control_result_t last_result;
    bool imu_ready;
    bool running;
} wheel_heading_control_snapshot_t;

void WheelHeadingControl_Init(uint32_t now_ms);
bool WheelHeadingControl_ConfigIsValid(
    const wheel_heading_control_config_t *config);
wheel_heading_control_result_t WheelHeadingControl_SetConfig(
    const wheel_heading_control_config_t *config);
bool WheelHeadingControl_GetConfig(
    wheel_heading_control_config_t *config);
wheel_heading_control_result_t WheelHeadingControl_StartAbsolute(
    float target_yaw_deg, float base_speed_pps, uint32_t now_ms);
wheel_heading_control_result_t WheelHeadingControl_StartHoldCurrent(
    float base_speed_pps, uint32_t now_ms);
wheel_heading_control_result_t WheelHeadingControl_SetCommand(
    float target_yaw_deg, float base_speed_pps, uint32_t now_ms);
/* Call after ICM20948_Task and before WheelSpeedControl_Task. */
void WheelHeadingControl_Task(uint32_t now_ms);
void WheelHeadingControl_Stop(car_control_block_reason_t reason);
bool WheelHeadingControl_GetSnapshot(
    wheel_heading_control_snapshot_t *snapshot);

#endif
