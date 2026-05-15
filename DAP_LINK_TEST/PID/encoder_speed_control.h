#ifndef PID_ENCODER_SPEED_CONTROL_H
#define PID_ENCODER_SPEED_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float kd;
} encoder_speed_control_pid_t;

void EncoderSpeedControl_Init(uint32_t now_ms);
void EncoderSpeedControl_Task(uint32_t now_ms);

void EncoderSpeedControl_SetTargetPps(
    float left_speed_pps, float right_speed_pps);
void EncoderSpeedControl_GetTargetPps(
    float *left_speed_pps, float *right_speed_pps);
void EncoderSpeedControl_SetMinDrivePwm(
    float left_pwm, float right_pwm, float reference_pps);
void EncoderSpeedControl_SetDirectionalMinDrivePwm(
    float left_forward_pwm, float left_reverse_pwm,
    float right_forward_pwm, float right_reverse_pwm,
    float reference_pps);
void EncoderSpeedControl_ClearMinDrivePwm(void);
void EncoderSpeedControl_GetSpeedTunings(
    encoder_speed_control_pid_t *left, encoder_speed_control_pid_t *right);
void EncoderSpeedControl_Stop(void);

#ifdef __cplusplus
}
#endif

#endif
