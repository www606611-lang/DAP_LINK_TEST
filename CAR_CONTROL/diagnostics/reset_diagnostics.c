#include "reset_diagnostics.h"

#include "ti_msp_dl_config.h"

static DL_SYSCTL_RESET_CAUSE g_reset_cause;

void ResetDiagnostics_Init(void)
{
    g_reset_cause = DL_SYSCTL_getResetCause();
}

uint32_t ResetDiagnostics_GetCause(void)
{
    return (uint32_t) g_reset_cause;
}

const char *ResetDiagnostics_GetCauseText(void)
{
    switch (g_reset_cause) {
        case DL_SYSCTL_RESET_CAUSE_NO_RESET:
            return "NO_RESET";
        case DL_SYSCTL_RESET_CAUSE_POR_HW_FAILURE:
            return "POR_HW_FAILURE";
        case DL_SYSCTL_RESET_CAUSE_POR_EXTERNAL_NRST:
            return "POR_EXTERNAL_NRST";
        case DL_SYSCTL_RESET_CAUSE_POR_SW_TRIGGERED:
            return "POR_SW_TRIGGERED";
        case DL_SYSCTL_RESET_CAUSE_BOR_SUPPLY_FAILURE:
            return "BOR_SUPPLY_FAILURE";
        case DL_SYSCTL_RESET_CAUSE_BOR_WAKE_FROM_SHUTDOWN:
            return "BOR_WAKE_SHUTDOWN";
        case DL_SYSCTL_RESET_CAUSE_BOOTRST_NON_PMU_PARITY_FAULT:
            return "BOOT_PARITY_FAULT";
        case DL_SYSCTL_RESET_CAUSE_BOOTRST_CLOCK_FAULT:
            return "BOOT_CLOCK_FAULT";
        case DL_SYSCTL_RESET_CAUSE_BOOTRST_SW_TRIGGERED:
            return "BOOT_SW_TRIGGERED";
        case DL_SYSCTL_RESET_CAUSE_BOOTRST_EXTERNAL_NRST:
            return "BOOT_EXTERNAL_NRST";
        case DL_SYSCTL_RESET_CAUSE_SYSRST_WWDT0_VIOLATION:
            return "SYS_WWDT0";
        case DL_SYSCTL_RESET_CAUSE_SYSRST_WWDT1_VIOLATION:
            return "SYS_WWDT1";
        case DL_SYSCTL_RESET_CAUSE_SYSRST_FLASH_ECC_ERROR:
            return "SYS_FLASH_ECC";
        case DL_SYSCTL_RESET_CAUSE_SYSRST_CPU_LOCKUP_VIOLATION:
            return "SYS_CPU_LOCKUP";
        case DL_SYSCTL_RESET_CAUSE_SYSRST_DEBUG_TRIGGERED:
            return "SYS_DEBUG";
        case DL_SYSCTL_RESET_CAUSE_SYSRST_SW_TRIGGERED:
            return "SYS_SW_TRIGGERED";
        case DL_SYSCTL_RESET_CAUSE_CPURST_DEBUG_TRIGGERED:
            return "CPU_DEBUG";
        case DL_SYSCTL_RESET_CAUSE_CPURST_SW_TRIGGERED:
            return "CPU_SW_TRIGGERED";
        default:
            return "OTHER_RESET";
    }
}

bool ResetDiagnostics_IsSuspicious(void)
{
    switch (g_reset_cause) {
        case DL_SYSCTL_RESET_CAUSE_BOR_SUPPLY_FAILURE:
        case DL_SYSCTL_RESET_CAUSE_BOOTRST_NON_PMU_PARITY_FAULT:
        case DL_SYSCTL_RESET_CAUSE_BOOTRST_CLOCK_FAULT:
        case DL_SYSCTL_RESET_CAUSE_SYSRST_WWDT0_VIOLATION:
        case DL_SYSCTL_RESET_CAUSE_SYSRST_WWDT1_VIOLATION:
        case DL_SYSCTL_RESET_CAUSE_SYSRST_FLASH_ECC_ERROR:
        case DL_SYSCTL_RESET_CAUSE_SYSRST_CPU_LOCKUP_VIOLATION:
            return true;
        default:
            return false;
    }
}
