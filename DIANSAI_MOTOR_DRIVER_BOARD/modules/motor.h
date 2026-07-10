#ifndef MODULES_MOTOR_H
#define MODULES_MOTOR_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 电机速度命令最大值，接口范围为 -1000 到 +1000。 */
#define MOTOR_PWM_MAX 1000

/* 电机续流方式：慢衰减低速更稳，快衰减响应更直接。 */
typedef enum {
    MOTOR_DECAY_SLOW = 0,
    MOTOR_DECAY_FAST
} motor_decay_t;

/* 电机编号：左电机和右电机。 */
typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT,
    MOTOR_ID_COUNT
} motor_id_t;

/* 初始化左右电机PWM并默认停止。 */
void Motor_Init(void);

/* 兼容旧接口：设置左电机速度。 */
void Motor_Set(int pwm);

/* 设置左电机速度，正负号表示方向。 */
void Motor_SetLeft(int pwm);

/* 设置右电机速度，正负号表示方向。 */
void Motor_SetRight(int pwm);

/* 停止左右电机，输出进入滑行状态。 */
void Motor_Stop(void);

/* 停止左电机，输出进入滑行状态。 */
void Motor_StopLeft(void);

/* 停止右电机，输出进入滑行状态。 */
void Motor_StopRight(void);

/* 左右电机同时刹车。 */
void Motor_Brake(void);

/* 左电机刹车。 */
void Motor_BrakeLeft(void);

/* 右电机刹车。 */
void Motor_BrakeRight(void);

/* 设置左右电机的续流方式。 */
void Motor_SetDecay(motor_decay_t decay);

/* 设置左电机的续流方式。 */
void Motor_SetLeftDecay(motor_decay_t decay);

/* 设置右电机的续流方式。 */
void Motor_SetRightDecay(motor_decay_t decay);

/* 兼容旧接口：设置左电机方向反相。 */
void Motor_SetInverted(bool inverted);

/* 设置左电机方向反相。 */
void Motor_SetLeftInverted(bool inverted);

/* 设置右电机方向反相。 */
void Motor_SetRightInverted(bool inverted);

/* 兼容旧接口：读取左电机当前速度命令。 */
int Motor_GetPwm(void);

/* 读取左电机当前速度命令。 */
int Motor_GetLeftPwm(void);

/* 读取右电机当前速度命令。 */
int Motor_GetRightPwm(void);

#ifdef __cplusplus
}
#endif

#endif
