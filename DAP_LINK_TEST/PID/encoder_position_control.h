#ifndef PID_ENCODER_POSITION_CONTROL_H
#define PID_ENCODER_POSITION_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float kd;
} encoder_position_control_pid_t;

typedef struct {
    int32_t count;
    int32_t speed_pps;
    float cascade_speed_target_pps;
    float pwm_command;
} encoder_position_control_state_t;

/* Call once after Motor_Init() and Encoder_Init(). */
void EncoderPositionControl_Init(uint32_t now_ms);

/* Call periodically after Encoder_Task(). */
void EncoderPositionControl_Task(uint32_t now_ms);

/* Absolute position target, in encoder counts. */
void EncoderPositionControl_SetTargetCount(
    float left_count, float right_count);

/* Relative move from the current target, in encoder counts. */
void EncoderPositionControl_AddTargetCount(
    float left_delta_count, float right_delta_count);

void EncoderPositionControl_GetTargetCount(
    float *left_count, float *right_count);
void EncoderPositionControl_GetCurrentCount(
    int32_t *left_count, int32_t *right_count);

/* Reset encoder counts and targets to zero. */
void EncoderPositionControl_ZeroPosition(uint32_t now_ms);

void EncoderPositionControl_GetState(
    encoder_position_control_state_t *left,
    encoder_position_control_state_t *right);
void EncoderPositionControl_GetPositionTunings(
    encoder_position_control_pid_t *left,
    encoder_position_control_pid_t *right);
void EncoderPositionControl_Stop(void);

#ifdef __cplusplus
}
#endif

#endif
