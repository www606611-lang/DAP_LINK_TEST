#ifndef CONTROL_WHEEL_SPEED_CONTROL_H
#define CONTROL_WHEEL_SPEED_CONTROL_H

#include "control_supervisor.h"

#include <stdbool.h>
#include <stdint.h>

#define WHEEL_SPEED_CONTROL_TARGET_MAX_PPS 6000.0f
#define WHEEL_SPEED_CONTROL_KP_MAX          5.0f
#define WHEEL_SPEED_CONTROL_KI_MAX         20.0f
#define WHEEL_SPEED_CONTROL_KD_MAX          2.0f
#define WHEEL_SPEED_CONTROL_COMMAND_LEASE_MS 100U

typedef enum {
    WHEEL_SPEED_CONTROL_OK = 0,
    WHEEL_SPEED_CONTROL_BAD_TARGET,
    WHEEL_SPEED_CONTROL_BAD_OUTPUT_LIMIT,
    WHEEL_SPEED_CONTROL_NOT_RUNNING,
    WHEEL_SPEED_CONTROL_SUPERVISOR_BLOCKED,
    WHEEL_SPEED_CONTROL_ENCODER_ERROR,
    WHEEL_SPEED_CONTROL_OUTPUT_ERROR,
    WHEEL_SPEED_CONTROL_BAD_TUNING,
    WHEEL_SPEED_CONTROL_BUSY,
    WHEEL_SPEED_CONTROL_BAD_OWNER,
    WHEEL_SPEED_CONTROL_COMMAND_TIMEOUT
} wheel_speed_control_result_t;

typedef struct {
    float kp;
    float ki;
    float kd;
} wheel_speed_control_tunings_t;

typedef struct {
    float left_target_pps;
    float right_target_pps;
    float left_error_pps;
    float right_error_pps;
    int32_t left_measured_pps;
    int32_t right_measured_pps;
    int16_t left_output_permille;
    int16_t right_output_permille;
    uint32_t update_count;
    uint32_t command_age_ms;
    wheel_speed_control_result_t last_result;
    car_control_mode_t owner_mode;
    bool running;
} wheel_speed_control_snapshot_t;

void WheelSpeedControl_Init(uint32_t now_ms);
bool WheelSpeedControl_TuningsAreValid(
    const wheel_speed_control_tunings_t *tunings);
wheel_speed_control_result_t WheelSpeedControl_SetTunings(
    const wheel_speed_control_tunings_t *tunings);
bool WheelSpeedControl_GetTunings(
    wheel_speed_control_tunings_t *tunings);
wheel_speed_control_result_t WheelSpeedControl_Start(uint32_t now_ms);
wheel_speed_control_result_t WheelSpeedControl_StartForMode(
    car_control_mode_t owner_mode, uint32_t now_ms);
wheel_speed_control_result_t WheelSpeedControl_SetTargets(
    float left_pps, float right_pps, uint32_t now_ms);
wheel_speed_control_result_t WheelSpeedControl_SetOutputLimits(
    uint16_t left_permille, uint16_t right_permille);
/* Call after EncoderInput_Task so the 10 ms filtered snapshot is current. */
void WheelSpeedControl_Task(uint32_t now_ms);
void WheelSpeedControl_Stop(car_control_block_reason_t reason);
bool WheelSpeedControl_GetSnapshot(
    wheel_speed_control_snapshot_t *snapshot);

#endif
