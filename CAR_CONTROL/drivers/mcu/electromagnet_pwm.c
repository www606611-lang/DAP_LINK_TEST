#include "electromagnet_pwm.h"

#include "ti_msp_dl_config.h"

static uint16_t g_duty_permille;

static void electromagnet_pwm_force_low(void);
static void electromagnet_pwm_force_gpio_high(void);
static void electromagnet_pwm_apply(uint16_t duty_permille);

void ElectromagnetPwm_ForceGpioLow(void)
{
    DL_GPIO_clearPins(
        GPIO_ELECTROMAGNET_PWM_C1_PORT,
        GPIO_ELECTROMAGNET_PWM_C1_PIN);
    DL_GPIO_initDigitalOutput(GPIO_ELECTROMAGNET_PWM_C1_IOMUX);
    DL_GPIO_enableOutput(
        GPIO_ELECTROMAGNET_PWM_C1_PORT,
        GPIO_ELECTROMAGNET_PWM_C1_PIN);
}

void ElectromagnetPwm_Init(void)
{
    DL_TimerG_stopCounter(ELECTROMAGNET_PWM_INST);
    electromagnet_pwm_force_low();
    ElectromagnetPwm_ForceGpioLow();
    g_duty_permille = 0U;
}

bool ElectromagnetPwm_SetDuty(uint16_t duty_permille)
{
    if (duty_permille > ELECTROMAGNET_PWM_DUTY_MAX) {
        return false;
    }

    if (duty_permille == 0U) {
        DL_TimerG_stopCounter(ELECTROMAGNET_PWM_INST);
        electromagnet_pwm_force_low();
        ElectromagnetPwm_ForceGpioLow();
    } else if (duty_permille == ELECTROMAGNET_PWM_DUTY_MAX) {
        DL_TimerG_stopCounter(ELECTROMAGNET_PWM_INST);
        electromagnet_pwm_force_gpio_high();
    } else {
        DL_TimerG_stopCounter(ELECTROMAGNET_PWM_INST);
        electromagnet_pwm_apply(duty_permille);
        DL_TimerG_setTimerCount(ELECTROMAGNET_PWM_INST,
            DL_TimerG_getLoadValue(ELECTROMAGNET_PWM_INST));
        DL_GPIO_initPeripheralOutputFunction(
            GPIO_ELECTROMAGNET_PWM_C1_IOMUX,
            GPIO_ELECTROMAGNET_PWM_C1_IOMUX_FUNC);
        DL_GPIO_enableOutput(
            GPIO_ELECTROMAGNET_PWM_C1_PORT,
            GPIO_ELECTROMAGNET_PWM_C1_PIN);
        DL_TimerG_startCounter(ELECTROMAGNET_PWM_INST);
    }
    g_duty_permille = duty_permille;
    return true;
}

uint16_t ElectromagnetPwm_GetDuty(void)
{
    return g_duty_permille;
}

static void electromagnet_pwm_force_low(void)
{
    DL_Timer_overrideCCPOut(ELECTROMAGNET_PWM_INST,
        DL_TIMER_FORCE_OUT_LOW, DL_TIMER_FORCE_CMPL_OUT_DISABLED,
        GPIO_ELECTROMAGNET_PWM_C1_IDX);
}

static void electromagnet_pwm_force_gpio_high(void)
{
    DL_GPIO_setPins(
        GPIO_ELECTROMAGNET_PWM_C1_PORT,
        GPIO_ELECTROMAGNET_PWM_C1_PIN);
    DL_GPIO_initDigitalOutput(GPIO_ELECTROMAGNET_PWM_C1_IOMUX);
    DL_GPIO_enableOutput(
        GPIO_ELECTROMAGNET_PWM_C1_PORT,
        GPIO_ELECTROMAGNET_PWM_C1_PIN);
}

static void electromagnet_pwm_apply(uint16_t duty_permille)
{
    uint32_t period =
        DL_TimerG_getLoadValue(ELECTROMAGNET_PWM_INST) + 1U;
    uint32_t high_ticks =
        (period * duty_permille) / ELECTROMAGNET_PWM_DUTY_MAX;

    DL_TimerG_setCaptureCompareValue(ELECTROMAGNET_PWM_INST,
        period - high_ticks, GPIO_ELECTROMAGNET_PWM_C1_IDX);
    DL_Timer_overrideCCPOut(ELECTROMAGNET_PWM_INST,
        DL_TIMER_FORCE_OUT_DISABLED,
        DL_TIMER_FORCE_CMPL_OUT_DISABLED,
        GPIO_ELECTROMAGNET_PWM_C1_IDX);
}
