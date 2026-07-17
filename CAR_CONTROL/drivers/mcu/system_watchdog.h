#ifndef DRIVERS_MCU_SYSTEM_WATCHDOG_H
#define DRIVERS_MCU_SYSTEM_WATCHDOG_H

#include <stdbool.h>
#include <stdint.h>

void SystemWatchdog_Init(void);
void SystemWatchdog_Kick(void);
void SystemWatchdog_PrepareForBootloader(void);
bool SystemWatchdog_StopKicksForTest(void);
bool SystemWatchdog_IsKickEnabled(void);
uint32_t SystemWatchdog_GetKickCount(void);

#endif
