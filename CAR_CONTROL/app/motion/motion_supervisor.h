#ifndef APP_MOTION_MOTION_SUPERVISOR_H
#define APP_MOTION_MOTION_SUPERVISOR_H

#include "control_supervisor.h"

#include <stdbool.h>
#include <stdint.h>

#define MOTION_SUPERVISOR_DEFAULT_OUTPUT_LIMIT_PERMILLE 750U

typedef enum {
    MOTION_SUPERVISOR_READY = 0,
    MOTION_SUPERVISOR_RUNNING,
    MOTION_SUPERVISOR_COMPLETE,
    MOTION_SUPERVISOR_ABORTED,
    MOTION_SUPERVISOR_LOCKED
} motion_supervisor_state_t;

typedef enum {
    MOTION_SUPERVISOR_OK = 0,
    MOTION_SUPERVISOR_BAD_ARGUMENT,
    MOTION_SUPERVISOR_BUSY,
    MOTION_SUPERVISOR_LOCKED_RESULT,
    MOTION_SUPERVISOR_SENSOR_ERROR,
    MOTION_SUPERVISOR_IMU_ERROR,
    MOTION_SUPERVISOR_SPEED_ERROR,
    MOTION_SUPERVISOR_TIMEOUT,
    MOTION_SUPERVISOR_STOPPED
} motion_supervisor_result_t;

typedef struct {
    motion_supervisor_state_t state;
    motion_supervisor_result_t last_result;
    int32_t target_delta_count;
    int32_t target_count;
    int32_t current_count;
    int32_t position_error_count;
    float target_heading_deg;
    float current_heading_deg;
    float heading_error_deg;
    float base_speed_target_pps;
    float heading_correction_pps;
    float left_speed_target_pps;
    float right_speed_target_pps;
    uint32_t elapsed_ms;
    uint32_t command_age_ms;
    uint32_t run_count;
    uint32_t update_count;
    uint32_t last_interval_ms;
    uint32_t max_interval_ms;
    uint16_t output_limit_permille;
    bool hold_heading;
    bool settled;
    bool running;
} motion_supervisor_snapshot_t;

void MotionSupervisor_Init(bool reset_locked);
void MotionSupervisor_Task(uint32_t now_ms);
bool MotionSupervisor_RequestRelative(
    int32_t delta_count, float target_heading_deg,
    float max_speed_pps, uint32_t timeout_ms);
bool MotionSupervisor_RequestRelativeHoldHeading(
    int32_t delta_count, float max_speed_pps, uint32_t timeout_ms);
void MotionSupervisor_RequestStop(void);
bool MotionSupervisor_IsActive(void);
const char *MotionSupervisor_GetStateText(void);
bool MotionSupervisor_GetSnapshot(
    motion_supervisor_snapshot_t *snapshot);

#endif
