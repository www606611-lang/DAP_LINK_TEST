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
    float kp;
    float ki;
    float kd;
    float output_min;
    float output_max;
    float integral_min;
    float integral_max;
    float deadband;
    float base_speed_pps;
    float left_pwm_limit;
    float right_pwm_limit;
} line_tracking_config_t;

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

/* Low-level line tracking controller. Prefer LineTrackingApp_* from main.c
 * when you want key/LCD/telemetry handling as well. */
void LineTrackingControl_Init(uint32_t now_ms);

/* Call periodically after Encoder_Task(). Produces speed-loop targets. */
void LineTrackingControl_Task(uint32_t now_ms);

void LineTrackingControl_Start(void);
void LineTrackingControl_Stop(void);
void LineTrackingControl_Toggle(void);
void LineTrackingControl_SetEnabled(bool enabled);
bool LineTrackingControl_IsEnabled(void);

/* Main runtime behavior knobs for route logic. */
void LineTrackingControl_SetBaseSpeedPps(float speed_pps);
void LineTrackingControl_SetMotorOutputEnabled(bool enabled);

/* Tuning/configuration APIs used by the Bluetooth PID console and store. */
void LineTrackingControl_SetTunings(float kp, float ki, float kd);
void LineTrackingControl_SetOutputLimits(float output_min, float output_max);
void LineTrackingControl_SetIntegralLimits(float integral_min,
    float integral_max);
void LineTrackingControl_SetDeadband(float deadband);
void LineTrackingControl_SetDriveOutputLimits(float left_pwm_limit,
    float right_pwm_limit);
void LineTrackingControl_GetTunings(line_tracking_pid_t *pid);
void LineTrackingControl_GetConfig(line_tracking_config_t *config);
void LineTrackingControl_GetState(line_tracking_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
