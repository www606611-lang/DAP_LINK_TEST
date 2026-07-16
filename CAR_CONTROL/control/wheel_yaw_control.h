#ifndef CONTROL_WHEEL_YAW_CONTROL_H
#define CONTROL_WHEEL_YAW_CONTROL_H

#include "control_supervisor.h"

#include <stdbool.h>
#include <stdint.h>

#define WHEEL_YAW_CONTROL_UPDATE_INTERVAL_MS 10U
#define WHEEL_YAW_CONTROL_RUN_MIN_MS         500U
#define WHEEL_YAW_CONTROL_RUN_MAX_MS       15000U
#define WHEEL_YAW_CONTROL_TARGET_MAX_DEG    180.0f

typedef enum {
    WHEEL_YAW_CONTROL_OK = 0,
    WHEEL_YAW_CONTROL_BAD_CONFIG,
    WHEEL_YAW_CONTROL_BAD_TARGET,
    WHEEL_YAW_CONTROL_BAD_TIMEOUT,
    WHEEL_YAW_CONTROL_NOT_RUNNING,
    WHEEL_YAW_CONTROL_BUSY,
    WHEEL_YAW_CONTROL_IMU_ERROR,
    WHEEL_YAW_CONTROL_SPEED_ERROR,
    WHEEL_YAW_CONTROL_TIMEOUT,
    WHEEL_YAW_CONTROL_STOPPED
} wheel_yaw_control_result_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float max_turn_speed_pps;
    float min_turn_speed_pps;
    float tolerance_deg;
    float settle_yaw_rate_dps;
    uint16_t settle_time_ms;
    uint16_t feedforward_boost_permille;
} wheel_yaw_control_config_t;

typedef struct {
    float target_yaw_deg;
    float current_yaw_deg;
    float error_deg;
    float yaw_rate_dps;
    float turn_speed_target_pps;
    float left_speed_target_pps;
    float right_speed_target_pps;
    uint32_t update_count;
    uint32_t elapsed_ms;
    wheel_yaw_control_result_t last_result;
    bool imu_ready;
    bool running;
    bool settled;
} wheel_yaw_control_snapshot_t;

void WheelYawControl_Init(uint32_t now_ms);
bool WheelYawControl_ConfigIsValid(
    const wheel_yaw_control_config_t *config);
wheel_yaw_control_result_t WheelYawControl_SetConfig(
    const wheel_yaw_control_config_t *config);
bool WheelYawControl_GetConfig(wheel_yaw_control_config_t *config);
wheel_yaw_control_result_t WheelYawControl_StartAbsolute(
    float target_yaw_deg, uint32_t timeout_ms, uint32_t now_ms);
wheel_yaw_control_result_t WheelYawControl_StartRelative(
    float delta_yaw_deg, uint32_t timeout_ms, uint32_t now_ms);
/* Call after ICM20948_Task and before WheelSpeedControl_Task. */
void WheelYawControl_Task(uint32_t now_ms);
void WheelYawControl_Stop(car_control_block_reason_t reason);
bool WheelYawControl_GetSnapshot(wheel_yaw_control_snapshot_t *snapshot);

#endif
