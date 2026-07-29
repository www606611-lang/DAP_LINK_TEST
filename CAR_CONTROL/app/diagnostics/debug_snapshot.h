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
    uint32_t app_state;
    uint32_t active_workflow;
    uint32_t control_mode;
    uint32_t control_block_reason;
    bool pb21_pressed;
    bool pb4_pressed;
    bool pb5_pressed;
    car_debug_button_id_t last_button_id;
    bool motor_high_impedance;
    bool imu_ready;
    bool imu_attitude_valid;
    uint32_t imu_sample_age_ms;
    uint32_t imu_read_error_count;
    int32_t imu_yaw_mdeg;
    int32_t imu_yaw_rate_mdps;
    int32_t encoder_0_count;
    int32_t encoder_1_count;
    int32_t encoder_count_difference;
    int32_t encoder_0_speed_pps;
    int32_t encoder_1_speed_pps;
    int32_t speed_left_target_pps;
    int32_t speed_right_target_pps;
    int32_t speed_left_output_permille;
    int32_t speed_right_output_permille;
    int32_t position_left_error_count;
    int32_t position_right_error_count;
    int32_t position_sync_correction_pps;
    int32_t yaw_target_mdeg;
    int32_t yaw_error_mdeg;
    int32_t yaw_turn_target_pps;
    uint32_t yaw_elapsed_ms;
    int32_t heading_base_target_pps;
    int32_t heading_error_mdeg;
    int32_t heading_correction_pps;
    int32_t line_tracking_base_target_pps;
    int32_t line_tracking_correction_pps;
    int32_t line_tracking_left_target_pps;
    int32_t line_tracking_right_target_pps;
    uint32_t line_tracking_elapsed_ms;
    uint32_t line_mission_state;
    int32_t motion_base_target_pps;
    int32_t motion_error_count;
    int32_t motion_heading_error_mdeg;
    uint32_t motion_elapsed_ms;
    uint32_t line_sensor_state;
    uint32_t line_sensor_active_mask;
    uint32_t line_sensor_active_count;
    uint32_t line_sensor_read_error_count;
    int32_t line_sensor_error;
    bool line_sensor_seen;
    bool line_sensor_ready;
    bool radio_online;
    bool radio_esp32_online;
    bool radio_k230_online;
    uint32_t radio_frame_age_ms;
    uint32_t radio_rx_frame_count;
    uint32_t radio_parse_error_count;
} car_debug_display_snapshot_t;

void CarDebugSnapshot_Update(void);
void CarDebugSnapshot_RecordButtonPress(car_debug_button_id_t button_id);
bool CarDebugSnapshot_GetDisplay(car_debug_display_snapshot_t *snapshot);

#endif
