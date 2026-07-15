#ifndef BSP_BOARD_MOTOR_SAFE_H
#define BSP_BOARD_MOTOR_SAFE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    BOARD_MOTOR_CHANNEL_A = 0,
    BOARD_MOTOR_CHANNEL_B,
    BOARD_MOTOR_CHANNEL_BOTH,
    BOARD_MOTOR_CHANNEL_COUNT
} board_motor_channel_t;

void BoardMotorSafe_Init(void);
void BoardMotorSafe_EmergencyStop(void);
bool BoardMotorSafe_Arm(board_motor_channel_t channel);

bool BoardMotorSafe_IsArmed(void);
bool BoardMotorSafe_IsHighImpedance(void);
board_motor_channel_t BoardMotorSafe_GetArmedChannel(void);

#endif
