#ifndef DRIVERS_DEVICE_ELECTROMAGNET_ELECTROMAGNET_H
#define DRIVERS_DEVICE_ELECTROMAGNET_ELECTROMAGNET_H

#include <stdbool.h>
#include <stdint.h>

#define ELECTROMAGNET_DEFAULT_PULL_IN_MS       200U
#define ELECTROMAGNET_DEFAULT_HOLD_PERMILLE   1000U
#define ELECTROMAGNET_PULL_IN_MIN_MS            50U
#define ELECTROMAGNET_PULL_IN_MAX_MS          1000U
#define ELECTROMAGNET_HOLD_MIN_PERMILLE        100U
#define ELECTROMAGNET_HOLD_MAX_PERMILLE       1000U
#define ELECTROMAGNET_MAX_PULSE_MS            2000U

typedef enum {
    ELECTROMAGNET_STATE_OFF = 0,
    ELECTROMAGNET_STATE_PULL_IN,
    ELECTROMAGNET_STATE_HOLD,
    ELECTROMAGNET_STATE_DIAGNOSTIC_ON,
    ELECTROMAGNET_STATE_DIAGNOSTIC_PULSE,
    ELECTROMAGNET_STATE_FAULT
} electromagnet_state_t;

typedef enum {
    ELECTROMAGNET_RESULT_OK = 0,
    ELECTROMAGNET_RESULT_BAD_DURATION,
    ELECTROMAGNET_RESULT_BAD_CONFIG,
    ELECTROMAGNET_RESULT_BUSY,
    ELECTROMAGNET_RESULT_OUTPUT_ERROR
} electromagnet_result_t;

typedef struct {
    uint16_t pull_in_ms;
    uint16_t hold_duty_permille;
} electromagnet_config_t;

typedef struct {
    electromagnet_state_t state;
    bool active;
    bool continuous;
    uint16_t duty_permille;
    uint16_t pull_in_ms;
    uint16_t hold_duty_permille;
    uint16_t requested_duration_ms;
    uint16_t remaining_ms;
    uint32_t grip_count;
    uint32_t release_count;
    uint32_t pulse_count;
    uint32_t continuous_on_count;
    uint32_t automatic_off_count;
    uint32_t fault_count;
} electromagnet_snapshot_t;

void Electromagnet_Init(void);
electromagnet_result_t Electromagnet_Grip(uint32_t now_ms);
void Electromagnet_Release(void);
electromagnet_result_t Electromagnet_SetConfig(
    const electromagnet_config_t *config);
bool Electromagnet_GetConfig(electromagnet_config_t *config);

electromagnet_result_t Electromagnet_On(void);
electromagnet_result_t Electromagnet_Pulse(
    uint16_t duration_ms, uint32_t now_ms);
void Electromagnet_Off(void);
void Electromagnet_Task(uint32_t now_ms);
bool Electromagnet_GetSnapshot(
    uint32_t now_ms, electromagnet_snapshot_t *snapshot);
const char *Electromagnet_GetStateText(electromagnet_state_t state);

#endif
