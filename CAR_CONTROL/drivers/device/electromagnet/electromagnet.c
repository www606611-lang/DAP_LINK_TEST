#include "electromagnet.h"

#include "electromagnet_pwm.h"

#include <stddef.h>

static electromagnet_state_t g_state;
static electromagnet_config_t g_config;
static uint16_t g_requested_duration_ms;
static uint32_t g_deadline_ms;
static uint32_t g_grip_count;
static uint32_t g_release_count;
static uint32_t g_pulse_count;
static uint32_t g_continuous_on_count;
static uint32_t g_automatic_off_count;
static uint32_t g_fault_count;

static bool electromagnet_deadline_reached(
    uint32_t now_ms, uint32_t deadline_ms);
static bool electromagnet_config_is_valid(
    const electromagnet_config_t *config);
static void electromagnet_enter_fault(void);

void Electromagnet_Init(void)
{
    ElectromagnetPwm_Init();
    g_state = ELECTROMAGNET_STATE_OFF;
    g_config.pull_in_ms = ELECTROMAGNET_DEFAULT_PULL_IN_MS;
    g_config.hold_duty_permille =
        ELECTROMAGNET_DEFAULT_HOLD_PERMILLE;
    g_requested_duration_ms = 0U;
    g_deadline_ms = 0U;
    g_grip_count = 0U;
    g_release_count = 0U;
    g_pulse_count = 0U;
    g_continuous_on_count = 0U;
    g_automatic_off_count = 0U;
    g_fault_count = 0U;
}

electromagnet_result_t Electromagnet_Grip(uint32_t now_ms)
{
    if (g_state != ELECTROMAGNET_STATE_OFF) {
        return ELECTROMAGNET_RESULT_BUSY;
    }
    if (!ElectromagnetPwm_SetDuty(ELECTROMAGNET_PWM_DUTY_MAX)) {
        electromagnet_enter_fault();
        return ELECTROMAGNET_RESULT_OUTPUT_ERROR;
    }

    g_state = ELECTROMAGNET_STATE_PULL_IN;
    g_requested_duration_ms = 0U;
    g_deadline_ms = now_ms + g_config.pull_in_ms;
    g_grip_count++;
    return ELECTROMAGNET_RESULT_OK;
}

void Electromagnet_Release(void)
{
    bool was_active = g_state != ELECTROMAGNET_STATE_OFF;

    (void) ElectromagnetPwm_SetDuty(0U);
    g_state = ELECTROMAGNET_STATE_OFF;
    g_requested_duration_ms = 0U;
    g_deadline_ms = 0U;
    if (was_active) {
        g_release_count++;
    }
}

electromagnet_result_t Electromagnet_SetConfig(
    const electromagnet_config_t *config)
{
    if (!electromagnet_config_is_valid(config)) {
        return ELECTROMAGNET_RESULT_BAD_CONFIG;
    }
    if (g_state != ELECTROMAGNET_STATE_OFF) {
        return ELECTROMAGNET_RESULT_BUSY;
    }

    g_config = *config;
    return ELECTROMAGNET_RESULT_OK;
}

bool Electromagnet_GetConfig(electromagnet_config_t *config)
{
    if (config == NULL) {
        return false;
    }
    *config = g_config;
    return true;
}

electromagnet_result_t Electromagnet_On(void)
{
    if (g_state != ELECTROMAGNET_STATE_OFF) {
        return ELECTROMAGNET_RESULT_BUSY;
    }
    if (!ElectromagnetPwm_SetDuty(ELECTROMAGNET_PWM_DUTY_MAX)) {
        electromagnet_enter_fault();
        return ELECTROMAGNET_RESULT_OUTPUT_ERROR;
    }

    g_state = ELECTROMAGNET_STATE_DIAGNOSTIC_ON;
    g_requested_duration_ms = 0U;
    g_deadline_ms = 0U;
    g_continuous_on_count++;
    return ELECTROMAGNET_RESULT_OK;
}

electromagnet_result_t Electromagnet_Pulse(
    uint16_t duration_ms, uint32_t now_ms)
{
    if ((duration_ms == 0U) ||
        (duration_ms > ELECTROMAGNET_MAX_PULSE_MS)) {
        Electromagnet_Release();
        return ELECTROMAGNET_RESULT_BAD_DURATION;
    }
    if (g_state != ELECTROMAGNET_STATE_OFF) {
        return ELECTROMAGNET_RESULT_BUSY;
    }
    if (!ElectromagnetPwm_SetDuty(ELECTROMAGNET_PWM_DUTY_MAX)) {
        electromagnet_enter_fault();
        return ELECTROMAGNET_RESULT_OUTPUT_ERROR;
    }

    g_state = ELECTROMAGNET_STATE_DIAGNOSTIC_PULSE;
    g_requested_duration_ms = duration_ms;
    g_deadline_ms = now_ms + duration_ms;
    g_pulse_count++;
    return ELECTROMAGNET_RESULT_OK;
}

void Electromagnet_Off(void)
{
    Electromagnet_Release();
}

void Electromagnet_Task(uint32_t now_ms)
{
    if (!electromagnet_deadline_reached(now_ms, g_deadline_ms)) {
        return;
    }

    if (g_state == ELECTROMAGNET_STATE_PULL_IN) {
        if (!ElectromagnetPwm_SetDuty(g_config.hold_duty_permille)) {
            electromagnet_enter_fault();
            return;
        }
        g_state = ELECTROMAGNET_STATE_HOLD;
        g_deadline_ms = 0U;
    } else if (g_state == ELECTROMAGNET_STATE_DIAGNOSTIC_PULSE) {
        Electromagnet_Release();
        g_automatic_off_count++;
    }
}

bool Electromagnet_GetSnapshot(
    uint32_t now_ms, electromagnet_snapshot_t *snapshot)
{
    uint32_t remaining_ms = 0U;
    bool timed_state;

    if (snapshot == NULL) {
        return false;
    }
    timed_state = (g_state == ELECTROMAGNET_STATE_PULL_IN) ||
        (g_state == ELECTROMAGNET_STATE_DIAGNOSTIC_PULSE);
    if (timed_state &&
        !electromagnet_deadline_reached(now_ms, g_deadline_ms)) {
        remaining_ms = g_deadline_ms - now_ms;
    }

    snapshot->state = g_state;
    snapshot->duty_permille = ElectromagnetPwm_GetDuty();
    snapshot->active = snapshot->duty_permille != 0U;
    snapshot->continuous =
        (g_state == ELECTROMAGNET_STATE_HOLD) ||
        (g_state == ELECTROMAGNET_STATE_DIAGNOSTIC_ON);
    snapshot->pull_in_ms = g_config.pull_in_ms;
    snapshot->hold_duty_permille = g_config.hold_duty_permille;
    snapshot->requested_duration_ms = g_requested_duration_ms;
    snapshot->remaining_ms = (uint16_t) remaining_ms;
    snapshot->grip_count = g_grip_count;
    snapshot->release_count = g_release_count;
    snapshot->pulse_count = g_pulse_count;
    snapshot->continuous_on_count = g_continuous_on_count;
    snapshot->automatic_off_count = g_automatic_off_count;
    snapshot->fault_count = g_fault_count;
    return true;
}

const char *Electromagnet_GetStateText(electromagnet_state_t state)
{
    switch (state) {
        case ELECTROMAGNET_STATE_OFF:
            return "OFF";
        case ELECTROMAGNET_STATE_PULL_IN:
            return "PULL_IN";
        case ELECTROMAGNET_STATE_HOLD:
            return "HOLD";
        case ELECTROMAGNET_STATE_DIAGNOSTIC_ON:
            return "ON";
        case ELECTROMAGNET_STATE_DIAGNOSTIC_PULSE:
            return "PULSE";
        case ELECTROMAGNET_STATE_FAULT:
            return "FAULT";
        default:
            return "UNKNOWN";
    }
}

static bool electromagnet_deadline_reached(
    uint32_t now_ms, uint32_t deadline_ms)
{
    return ((int32_t) (now_ms - deadline_ms) >= 0);
}

static bool electromagnet_config_is_valid(
    const electromagnet_config_t *config)
{
    return (config != NULL) &&
        (config->pull_in_ms >= ELECTROMAGNET_PULL_IN_MIN_MS) &&
        (config->pull_in_ms <= ELECTROMAGNET_PULL_IN_MAX_MS) &&
        (config->hold_duty_permille >=
            ELECTROMAGNET_HOLD_MIN_PERMILLE) &&
        (config->hold_duty_permille <=
            ELECTROMAGNET_HOLD_MAX_PERMILLE);
}

static void electromagnet_enter_fault(void)
{
    (void) ElectromagnetPwm_SetDuty(0U);
    g_state = ELECTROMAGNET_STATE_FAULT;
    g_requested_duration_ms = 0U;
    g_deadline_ms = 0U;
    g_fault_count++;
}
