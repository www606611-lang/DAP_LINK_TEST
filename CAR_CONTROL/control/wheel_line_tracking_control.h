#ifndef CONTROL_WHEEL_LINE_TRACKING_CONTROL_H
#define CONTROL_WHEEL_LINE_TRACKING_CONTROL_H

#include "control_supervisor.h"

#include <stdbool.h>
#include <stdint.h>

#define WHEEL_LINE_TRACKING_UPDATE_INTERVAL_MS 10U
#define WHEEL_LINE_TRACKING_COMMAND_LEASE_MS  100U
#define WHEEL_LINE_TRACKING_OBSERVATION_MAX_AGE_MS 60U

typedef enum {
    WHEEL_LINE_TRACKING_OK = 0,
    WHEEL_LINE_TRACKING_BAD_CONFIG,
    WHEEL_LINE_TRACKING_BAD_COMMAND,
    WHEEL_LINE_TRACKING_NOT_RUNNING,
    WHEEL_LINE_TRACKING_BUSY,
    WHEEL_LINE_TRACKING_LINE_LOST,
    WHEEL_LINE_TRACKING_SENSOR_STALE,
    WHEEL_LINE_TRACKING_SPEED_ERROR,
    WHEEL_LINE_TRACKING_COMMAND_TIMEOUT,
    WHEEL_LINE_TRACKING_STOPPED
} wheel_line_tracking_result_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float max_correction_pps;
    float deadband;
} wheel_line_tracking_config_t;

typedef struct {
    int16_t line_error;
    uint8_t active_count;
    float base_speed_target_pps;
    float correction_target_pps;
    float left_speed_target_pps;
    float right_speed_target_pps;
    float target_yaw_rate_dps;
    float measured_yaw_rate_dps;
    float yaw_rate_boost_pps;
    uint32_t update_count;
    uint32_t elapsed_ms;
    uint32_t command_age_ms;
    uint32_t observation_age_ms;
    uint32_t last_interval_ms;
    uint32_t max_interval_ms;
    wheel_line_tracking_result_t last_result;
    bool imu_feedback_valid;
    bool line_seen;
    bool running;
} wheel_line_tracking_snapshot_t;

void WheelLineTrackingControl_Init(uint32_t now_ms);
bool WheelLineTrackingControl_ConfigIsValid(
    const wheel_line_tracking_config_t *config);
wheel_line_tracking_result_t WheelLineTrackingControl_SetConfig(
    const wheel_line_tracking_config_t *config);
bool WheelLineTrackingControl_GetConfig(
    wheel_line_tracking_config_t *config);
wheel_line_tracking_result_t WheelLineTrackingControl_Start(
    float base_speed_pps, int16_t line_error, uint8_t active_count,
    bool line_seen,
    uint32_t observation_ms, uint32_t now_ms);
wheel_line_tracking_result_t WheelLineTrackingControl_SetCommand(
    float base_speed_pps, int16_t line_error, uint8_t active_count,
    bool line_seen,
    uint32_t observation_ms, uint32_t now_ms);
void WheelLineTrackingControl_Task(uint32_t now_ms);
void WheelLineTrackingControl_Stop(car_control_block_reason_t reason);
bool WheelLineTrackingControl_GetSnapshot(
    wheel_line_tracking_snapshot_t *snapshot);

#endif
