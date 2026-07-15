#include "at8236_motor.h"

#include "motor_pwm.h"

static int16_t g_command[AT8236_MOTOR_COUNT];
static bool g_inverted[AT8236_MOTOR_COUNT];

static bool at8236_motor_is_valid_id(at8236_motor_id_t id);
static motor_pwm_channel_t at8236_motor_get_pwm_channel(
    at8236_motor_id_t id);

void AT8236_MotorInit(void)
{
    g_command[AT8236_MOTOR_A] = 0;
    g_command[AT8236_MOTOR_B] = 0;
    g_inverted[AT8236_MOTOR_A] = false;
    g_inverted[AT8236_MOTOR_B] = false;
}

void AT8236_MotorSetInverted(at8236_motor_id_t id, bool inverted)
{
    if (!at8236_motor_is_valid_id(id)) {
        return;
    }
    g_inverted[id] = inverted;
}

bool AT8236_MotorSetCommand(
    at8236_motor_id_t id, int16_t command_permille)
{
    motor_pwm_channel_t channel;
    int16_t hardware_command;
    uint16_t magnitude;
    bool applied;

    if ((!at8236_motor_is_valid_id(id)) ||
        (command_permille > AT8236_MOTOR_COMMAND_MAX) ||
        (command_permille < -AT8236_MOTOR_COMMAND_MAX)) {
        return false;
    }

    channel = at8236_motor_get_pwm_channel(id);
    hardware_command = g_inverted[id] ?
        (int16_t) -command_permille : command_permille;
    magnitude = (uint16_t) ((hardware_command < 0) ?
        -hardware_command : hardware_command);
    if (hardware_command > 0) {
        applied = MotorPwm_SetDuty(channel, magnitude, 0U);
    } else if (hardware_command < 0) {
        applied = MotorPwm_SetDuty(channel, 0U, magnitude);
    } else {
        applied = MotorPwm_SetDuty(channel, 0U, 0U);
    }

    if (applied) {
        g_command[id] = command_permille;
    }
    return applied;
}

void AT8236_MotorStop(at8236_motor_id_t id)
{
    if (!at8236_motor_is_valid_id(id)) {
        return;
    }
    (void) MotorPwm_SetDuty(
        at8236_motor_get_pwm_channel(id), 0U, 0U);
    g_command[id] = 0;
}

void AT8236_MotorStopAll(void)
{
    AT8236_MotorStop(AT8236_MOTOR_A);
    AT8236_MotorStop(AT8236_MOTOR_B);
}

int16_t AT8236_MotorGetCommand(at8236_motor_id_t id)
{
    if (!at8236_motor_is_valid_id(id)) {
        return 0;
    }
    return g_command[id];
}

static bool at8236_motor_is_valid_id(at8236_motor_id_t id)
{
    return ((uint32_t) id < (uint32_t) AT8236_MOTOR_COUNT);
}

static motor_pwm_channel_t at8236_motor_get_pwm_channel(
    at8236_motor_id_t id)
{
    return (id == AT8236_MOTOR_A) ?
        MOTOR_PWM_CHANNEL_A : MOTOR_PWM_CHANNEL_B;
}
