#ifndef CONTROL_WHEEL_POSITION_CONTROL_H
#define CONTROL_WHEEL_POSITION_CONTROL_H

#include "control_supervisor.h"

#include <stdbool.h>
#include <stdint.h>

#define WHEEL_POSITION_CONTROL_UPDATE_INTERVAL_MS 20U
#define WHEEL_POSITION_CONTROL_TARGET_MAX_COUNTS  100000000L
#define WHEEL_POSITION_CONTROL_RUN_MIN_MS         500U
#define WHEEL_POSITION_CONTROL_RUN_MAX_MS         30000U
#define WHEEL_POSITION_CONTROL_KP_MAX             20.0f

typedef enum {
    WHEEL_POSITION_CONTROL_OK = 0,
    WHEEL_POSITION_CONTROL_BAD_CONFIG,
    WHEEL_POSITION_CONTROL_BAD_TARGET,
    WHEEL_POSITION_CONTROL_BAD_TIMEOUT,
    WHEEL_POSITION_CONTROL_NOT_RUNNING,
    WHEEL_POSITION_CONTROL_BUSY,
    WHEEL_POSITION_CONTROL_ENCODER_ERROR,
    WHEEL_POSITION_CONTROL_SPEED_ERROR,
    WHEEL_POSITION_CONTROL_TIMEOUT,
    WHEEL_POSITION_CONTROL_STOPPED
} wheel_position_control_result_t;

typedef struct {
    float kp;
    float max_speed_pps;
    uint16_t tolerance_counts;
    uint16_t settle_speed_pps;
    uint16_t settle_time_ms;
} wheel_position_control_config_t;

typedef struct {
    int32_t left_target_count;
    int32_t right_target_count;
    int32_t left_count;
    int32_t right_count;
    int32_t left_error_count;
    int32_t right_error_count;
    float left_speed_target_pps;
    float right_speed_target_pps;
    uint32_t left_recovery_count;
    uint32_t right_recovery_count;
    uint32_t update_count;
    uint32_t elapsed_ms;
    wheel_position_control_result_t last_result;
    bool running;
    bool settled;
    bool left_recovery_active;
    bool right_recovery_active;
} wheel_position_control_snapshot_t;

void WheelPositionControl_Init(uint32_t now_ms);
bool WheelPositionControl_ConfigIsValid(
    const wheel_position_control_config_t *config);
wheel_position_control_result_t WheelPositionControl_SetConfig(
    const wheel_position_control_config_t *config);
bool WheelPositionControl_GetConfig(
    wheel_position_control_config_t *config);
wheel_position_control_result_t WheelPositionControl_StartAbsolute(
    int32_t left_target_count, int32_t right_target_count,
    uint32_t timeout_ms, uint32_t now_ms);
wheel_position_control_result_t WheelPositionControl_StartRelative(
    int32_t left_delta_count, int32_t right_delta_count,
    uint32_t timeout_ms, uint32_t now_ms);
/* Call after EncoderInput_Task and before WheelSpeedControl_Task. */
void WheelPositionControl_Task(uint32_t now_ms);
void WheelPositionControl_Stop(car_control_block_reason_t reason);
bool WheelPositionControl_GetSnapshot(
    wheel_position_control_snapshot_t *snapshot);

#endif
