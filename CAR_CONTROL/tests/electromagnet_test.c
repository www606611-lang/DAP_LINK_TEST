#include "electromagnet.h"
#include "electromagnet_pwm.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

static uint16_t g_pwm_duty;
static bool g_pwm_accept;

void ElectromagnetPwm_ForceGpioLow(void)
{
    g_pwm_duty = 0U;
}

void ElectromagnetPwm_Init(void)
{
    g_pwm_duty = 0U;
    g_pwm_accept = true;
}

bool ElectromagnetPwm_SetDuty(uint16_t duty_permille)
{
    if (!g_pwm_accept ||
        (duty_permille > ELECTROMAGNET_PWM_DUTY_MAX)) {
        return false;
    }
    g_pwm_duty = duty_permille;
    return true;
}

uint16_t ElectromagnetPwm_GetDuty(void)
{
    return g_pwm_duty;
}

static electromagnet_snapshot_t snapshot_at(uint32_t now_ms)
{
    electromagnet_snapshot_t snapshot;

    assert(Electromagnet_GetSnapshot(now_ms, &snapshot));
    return snapshot;
}

static void test_defaults_are_off(void)
{
    electromagnet_snapshot_t snapshot;
    electromagnet_config_t config;

    Electromagnet_Init();
    snapshot = snapshot_at(0U);
    assert(snapshot.state == ELECTROMAGNET_STATE_OFF);
    assert(!snapshot.active);
    assert(snapshot.duty_permille == 0U);
    assert(Electromagnet_GetConfig(&config));
    assert(config.pull_in_ms == ELECTROMAGNET_DEFAULT_PULL_IN_MS);
    assert(config.hold_duty_permille ==
        ELECTROMAGNET_DEFAULT_HOLD_PERMILLE);
}

static void test_grip_transitions_to_hold(void)
{
    electromagnet_snapshot_t snapshot;

    Electromagnet_Init();
    assert(Electromagnet_Grip(1000U) == ELECTROMAGNET_RESULT_OK);
    snapshot = snapshot_at(1000U);
    assert(snapshot.state == ELECTROMAGNET_STATE_PULL_IN);
    assert(snapshot.active);
    assert(!snapshot.continuous);
    assert(snapshot.duty_permille == ELECTROMAGNET_PWM_DUTY_MAX);
    assert(snapshot.remaining_ms == ELECTROMAGNET_DEFAULT_PULL_IN_MS);
    assert(snapshot.grip_count == 1U);

    Electromagnet_Task(1199U);
    assert(g_pwm_duty == ELECTROMAGNET_PWM_DUTY_MAX);
    Electromagnet_Task(1200U);
    snapshot = snapshot_at(1200U);
    assert(snapshot.state == ELECTROMAGNET_STATE_HOLD);
    assert(snapshot.active);
    assert(snapshot.continuous);
    assert(snapshot.duty_permille ==
        ELECTROMAGNET_DEFAULT_HOLD_PERMILLE);

    Electromagnet_Task(UINT32_MAX);
    assert(g_pwm_duty == ELECTROMAGNET_DEFAULT_HOLD_PERMILLE);
    Electromagnet_Release();
    snapshot = snapshot_at(0U);
    assert(snapshot.state == ELECTROMAGNET_STATE_OFF);
    assert(!snapshot.active);
    assert(snapshot.release_count == 1U);
}

static void test_config_validation_and_custom_hold(void)
{
    electromagnet_config_t config = {250U, 400U};
    electromagnet_snapshot_t snapshot;

    Electromagnet_Init();
    assert(Electromagnet_SetConfig(NULL) ==
        ELECTROMAGNET_RESULT_BAD_CONFIG);
    config.pull_in_ms = ELECTROMAGNET_PULL_IN_MIN_MS - 1U;
    assert(Electromagnet_SetConfig(&config) ==
        ELECTROMAGNET_RESULT_BAD_CONFIG);
    config.pull_in_ms = 250U;
    config.hold_duty_permille = ELECTROMAGNET_HOLD_MAX_PERMILLE + 1U;
    assert(Electromagnet_SetConfig(&config) ==
        ELECTROMAGNET_RESULT_BAD_CONFIG);

    config.hold_duty_permille = 400U;
    assert(Electromagnet_SetConfig(&config) == ELECTROMAGNET_RESULT_OK);
    assert(Electromagnet_Grip(0U) == ELECTROMAGNET_RESULT_OK);
    assert(Electromagnet_SetConfig(&config) == ELECTROMAGNET_RESULT_BUSY);
    Electromagnet_Task(249U);
    assert(g_pwm_duty == ELECTROMAGNET_PWM_DUTY_MAX);
    Electromagnet_Task(250U);
    snapshot = snapshot_at(250U);
    assert(snapshot.state == ELECTROMAGNET_STATE_HOLD);
    assert(snapshot.duty_permille == 400U);
    Electromagnet_Release();
}

static void test_diagnostic_modes(void)
{
    electromagnet_snapshot_t snapshot;

    Electromagnet_Init();
    assert(Electromagnet_On() == ELECTROMAGNET_RESULT_OK);
    snapshot = snapshot_at(100U);
    assert(snapshot.state == ELECTROMAGNET_STATE_DIAGNOSTIC_ON);
    assert(snapshot.continuous);
    assert(snapshot.duty_permille == ELECTROMAGNET_PWM_DUTY_MAX);
    assert(Electromagnet_On() == ELECTROMAGNET_RESULT_BUSY);
    Electromagnet_Release();

    assert(Electromagnet_Pulse(100U, 1000U) == ELECTROMAGNET_RESULT_OK);
    snapshot = snapshot_at(1000U);
    assert(snapshot.state == ELECTROMAGNET_STATE_DIAGNOSTIC_PULSE);
    assert(snapshot.remaining_ms == 100U);
    Electromagnet_Task(1099U);
    assert(g_pwm_duty == ELECTROMAGNET_PWM_DUTY_MAX);
    Electromagnet_Task(1100U);
    snapshot = snapshot_at(1100U);
    assert(snapshot.state == ELECTROMAGNET_STATE_OFF);
    assert(snapshot.automatic_off_count == 1U);
}

static void test_invalid_pulse_forces_release(void)
{
    Electromagnet_Init();
    assert(Electromagnet_On() == ELECTROMAGNET_RESULT_OK);
    assert(Electromagnet_Pulse(0U, 20U) ==
        ELECTROMAGNET_RESULT_BAD_DURATION);
    assert(g_pwm_duty == 0U);
    assert(Electromagnet_Pulse(ELECTROMAGNET_MAX_PULSE_MS + 1U, 30U) ==
        ELECTROMAGNET_RESULT_BAD_DURATION);
}

static void test_millisecond_counter_wrap(void)
{
    electromagnet_snapshot_t snapshot;

    Electromagnet_Init();
    assert(Electromagnet_Grip(UINT32_MAX - 99U) ==
        ELECTROMAGNET_RESULT_OK);
    Electromagnet_Task(99U);
    assert(g_pwm_duty == ELECTROMAGNET_PWM_DUTY_MAX);
    Electromagnet_Task(100U);
    snapshot = snapshot_at(100U);
    assert(snapshot.state == ELECTROMAGNET_STATE_HOLD);
    assert(snapshot.duty_permille ==
        ELECTROMAGNET_DEFAULT_HOLD_PERMILLE);
}

static void test_output_failure_latches_fault(void)
{
    electromagnet_snapshot_t snapshot;

    Electromagnet_Init();
    g_pwm_accept = false;
    assert(Electromagnet_Grip(0U) == ELECTROMAGNET_RESULT_OUTPUT_ERROR);
    snapshot = snapshot_at(0U);
    assert(snapshot.state == ELECTROMAGNET_STATE_FAULT);
    assert(snapshot.fault_count == 1U);
    g_pwm_accept = true;
    Electromagnet_Release();
    assert(snapshot_at(0U).state == ELECTROMAGNET_STATE_OFF);
}

int main(void)
{
    test_defaults_are_off();
    test_grip_transitions_to_hold();
    test_config_validation_and_custom_hold();
    test_diagnostic_modes();
    test_invalid_pulse_forces_release();
    test_millisecond_counter_wrap();
    test_output_failure_latches_fault();
    return 0;
}
