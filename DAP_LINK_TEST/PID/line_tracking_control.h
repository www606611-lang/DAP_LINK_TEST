#ifndef PID_LINE_TRACKING_CONTROL_H
#define PID_LINE_TRACKING_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float kd;
} line_tracking_pid_t;

typedef struct {
    uint8_t raw;
    uint8_t active_mask;
    uint8_t active_count;
    uint8_t sensor_error;
    int16_t line_error;
    float turn_correction_pps;
    float base_speed_pps;
    float left_target_pps;
    float right_target_pps;
    int32_t left_actual_pps;
    int32_t right_actual_pps;
    bool sensor_ok;
    bool line_seen;
    bool enabled;
} line_tracking_state_t;

void LineTrackingControl_Init(uint32_t now_ms);
void LineTrackingControl_Task(uint32_t now_ms);
void LineTrackingControl_Start(void);
void LineTrackingControl_Stop(void);
void LineTrackingControl_Toggle(void);
void LineTrackingControl_SetEnabled(bool enabled);
bool LineTrackingControl_IsEnabled(void);
void LineTrackingControl_SetBaseSpeedPps(float speed_pps);
void LineTrackingControl_SetTunings(float kp, float ki, float kd);
void LineTrackingControl_GetTunings(line_tracking_pid_t *pid);
void LineTrackingControl_GetState(line_tracking_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
