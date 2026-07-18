#ifndef DIAGNOSTICS_DEBUG_SNAPSHOT_H
#define DIAGNOSTICS_DEBUG_SNAPSHOT_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    CAR_DEBUG_BUTTON_NONE = 0,
    CAR_DEBUG_BUTTON_PB4 = 4,
    CAR_DEBUG_BUTTON_PB5 = 5,
    CAR_DEBUG_BUTTON_PB21 = 21
} car_debug_button_id_t;

typedef struct {
    bool pb21_pressed;
    bool pb4_pressed;
    bool pb5_pressed;
    int32_t last_button_yaw_mdeg;
    car_debug_button_id_t last_button_id;
    bool motor_high_impedance;
    bool imu_ready;
    bool imu_attitude_valid;
    int32_t imu_yaw_mdeg;
    int32_t imu_yaw_rate_mdps;
    int32_t yaw_target_mdeg;
    int32_t yaw_error_mdeg;
    uint32_t yaw_elapsed_ms;
    int32_t heading_base_target_pps;
    int32_t line_tracking_base_target_pps;
    uint32_t line_tracking_elapsed_ms;
    int32_t motion_base_target_pps;
    uint32_t motion_elapsed_ms;
    uint32_t line_sensor_state;
    uint32_t line_sensor_active_mask;
    int32_t line_sensor_error;
    bool line_sensor_seen;
    bool line_sensor_ready;
} car_debug_display_snapshot_t;

void CarDebugSnapshot_Update(void);
void CarDebugSnapshot_RecordButtonPress(car_debug_button_id_t button_id);
void CarDebugSnapshot_SetButtonYawCommand(int32_t yaw_command_mdeg);
bool CarDebugSnapshot_GetDisplay(car_debug_display_snapshot_t *snapshot);

#endif
