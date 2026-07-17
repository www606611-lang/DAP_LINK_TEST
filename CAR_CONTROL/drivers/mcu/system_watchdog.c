#include "system_watchdog.h"

#include "ti_msp_dl_config.h"

static uint32_t g_kick_count;
static bool g_kick_enabled;

void SystemWatchdog_Init(void)
{
    DL_WWDT_setCoreHaltBehavior(
        APP_WWDT_INST, DL_WWDT_CORE_HALT_STOP);
    g_kick_count = 0U;
    g_kick_enabled = true;
    SystemWatchdog_Kick();
}

void SystemWatchdog_Kick(void)
{
    if (!g_kick_enabled) {
        return;
    }
    DL_WWDT_restart(APP_WWDT_INST);
    g_kick_count++;
}

void SystemWatchdog_PrepareForBootloader(void)
{
    g_kick_enabled = false;
    DL_WWDT_reset(APP_WWDT_INST);
    DL_WWDT_disablePower(APP_WWDT_INST);
}

bool SystemWatchdog_StopKicksForTest(void)
{
    if (!g_kick_enabled) {
        return false;
    }
    g_kick_enabled = false;
    return true;
}

bool SystemWatchdog_IsKickEnabled(void)
{
    return g_kick_enabled;
}

uint32_t SystemWatchdog_GetKickCount(void)
{
    return g_kick_count;
}
