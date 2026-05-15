#ifndef PID_YAW_ANGLE_CONTROL_H
#define PID_YAW_ANGLE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float kd;
} yaw_angle_control_pid_t;

typedef struct {
    float target_yaw_deg;
    float current_yaw_deg;
    float error_deg;
    float turn_speed_target_pps;
    float left_speed_target_pps;
    float right_speed_target_pps;
    bool imu_ready;
    bool enabled;
} yaw_angle_control_state_t;

/* Call once after ICM20948_TaskInit(), Motor_Init(), and Encoder_Init(). */
void YawAngleControl_Init(uint32_t now_ms);

/* Call periodically after ICM20948_Task() and Encoder_Task(). */
void YawAngleControl_Task(uint32_t now_ms);

/* Absolute yaw target in degrees, wrapped to [-180, 180]. */
void YawAngleControl_SetTargetDeg(float yaw_deg);

/* Relative yaw move from the current target, in degrees. */
void YawAngleControl_AddTargetDeg(float delta_yaw_deg);

/* Set the target to the current IMU yaw. */
void YawAngleControl_HoldCurrentYaw(void);
void YawAngleControl_GetTargetDeg(float *yaw_deg);

/* Reset IMU yaw angle and yaw target to zero. */
void YawAngleControl_ZeroYaw(uint32_t now_ms);

void YawAngleControl_GetState(yaw_angle_control_state_t *state);
void YawAngleControl_GetTunings(yaw_angle_control_pid_t *pid);
void YawAngleControl_Stop(void);

#ifdef __cplusplus
}
#endif

#endif
