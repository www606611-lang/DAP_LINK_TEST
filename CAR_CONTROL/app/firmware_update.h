#ifndef APP_FIRMWARE_UPDATE_H
#define APP_FIRMWARE_UPDATE_H

#include <stdbool.h>

void FirmwareUpdate_AppInit(void);
bool FirmwareUpdate_RequestBootloader(void);
bool FirmwareUpdate_IsPending(void);
void FirmwareUpdate_Task(void);

#endif
