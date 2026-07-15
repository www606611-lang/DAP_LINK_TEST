#ifndef BSP_BOARD_MOTOR_SAFE_H
#define BSP_BOARD_MOTOR_SAFE_H

#include <stdbool.h>

void BoardMotorSafe_Init(void);
void BoardMotorSafe_EmergencyStop(void);

bool BoardMotorSafe_IsArmed(void);
bool BoardMotorSafe_IsHighImpedance(void);

#endif
