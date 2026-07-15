#include "board_wheel_drive.h"

#include "at8236_motor.h"
#include "board_motor_safe.h"
#include "board_resources.h"

#include <stdbool.h>

#if BOARD_WHEEL_DRIVE_COMMAND_MAX != AT8236_MOTOR_COMMAND_MAX
#error "Board and AT8236 command ranges must match"
#endif

static bool board_wheel_drive_command_is_valid(int16_t command_permille)
{
    return (command_permille >= -BOARD_WHEEL_DRIVE_COMMAND_MAX) &&
        (command_permille <= BOARD_WHEEL_DRIVE_COMMAND_MAX);
}

void BoardWheelDrive_Init(void)
{
    AT8236_MotorInit();
    AT8236_MotorSetInverted(AT8236_MOTOR_A,
        BOARD_MOTOR_A_FORWARD_INVERTED != 0);
    AT8236_MotorSetInverted(AT8236_MOTOR_B,
        BOARD_MOTOR_B_FORWARD_INVERTED != 0);
}

board_wheel_drive_result_t BoardWheelDrive_SetCommands(
    int16_t left_permille, int16_t right_permille)
{
    if (!board_wheel_drive_command_is_valid(left_permille) ||
        !board_wheel_drive_command_is_valid(right_permille)) {
        BoardWheelDrive_SetZero();
        return BOARD_WHEEL_DRIVE_BAD_COMMAND;
    }

    if (!BoardMotorSafe_IsArmed() ||
        (BoardMotorSafe_GetArmedChannel() !=
            BOARD_MOTOR_CHANNEL_BOTH)) {
        BoardWheelDrive_SetZero();
        return BOARD_WHEEL_DRIVE_NOT_ARMED;
    }

    if (!AT8236_MotorSetCommand(AT8236_MOTOR_A, left_permille)) {
        BoardWheelDrive_SetZero();
        return BOARD_WHEEL_DRIVE_OUTPUT_ERROR;
    }
    if (!AT8236_MotorSetCommand(AT8236_MOTOR_B, right_permille)) {
        BoardWheelDrive_SetZero();
        return BOARD_WHEEL_DRIVE_OUTPUT_ERROR;
    }

    return BOARD_WHEEL_DRIVE_OK;
}

void BoardWheelDrive_SetZero(void)
{
    AT8236_MotorStopAll();
}

int16_t BoardWheelDrive_GetLeftCommand(void)
{
    return AT8236_MotorGetCommand(AT8236_MOTOR_A);
}

int16_t BoardWheelDrive_GetRightCommand(void)
{
    return AT8236_MotorGetCommand(AT8236_MOTOR_B);
}
