#ifndef APP_PID_TUNING_STORE_H
#define APP_PID_TUNING_STORE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PID_TUNING_STORE_OK = 0U,
    PID_TUNING_STORE_NOT_FOUND,
    PID_TUNING_STORE_BAD_CRC,
    PID_TUNING_STORE_BAD_SIZE,
    PID_TUNING_STORE_FLASH_ERROR
} pid_tuning_store_status_t;

pid_tuning_store_status_t PidTuningStore_SaveCurrent(void);
pid_tuning_store_status_t PidTuningStore_LoadApply(void);
pid_tuning_store_status_t PidTuningStore_Clear(void);
pid_tuning_store_status_t PidTuningStore_GetStatus(void);
const char *PidTuningStore_StatusText(pid_tuning_store_status_t status);
bool PidTuningStore_HasValidImage(void);

#ifdef __cplusplus
}
#endif

#endif
