#ifndef BSP_BOARD_WHEEL_DRIVE_H
#define BSP_BOARD_WHEEL_DRIVE_H

#include <stdint.h>

#define BOARD_WHEEL_DRIVE_COMMAND_MAX 1000

typedef enum {
    BOARD_WHEEL_DRIVE_OK = 0,
    BOARD_WHEEL_DRIVE_BAD_COMMAND,
    BOARD_WHEEL_DRIVE_NOT_ARMED,
    BOARD_WHEEL_DRIVE_OUTPUT_ERROR
} board_wheel_drive_result_t;

void BoardWheelDrive_Init(void);
board_wheel_drive_result_t BoardWheelDrive_SetCommands(
    int16_t left_permille, int16_t right_permille);
void BoardWheelDrive_SetZero(void);
int16_t BoardWheelDrive_GetLeftCommand(void);
int16_t BoardWheelDrive_GetRightCommand(void);

#endif
