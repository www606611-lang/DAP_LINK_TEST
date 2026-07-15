#ifndef APP_MOTOR_BRINGUP_TEST_H
#define APP_MOTOR_BRINGUP_TEST_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MOTOR_BRINGUP_TEST_LOCKED = 0,
    MOTOR_BRINGUP_TEST_READY,
    MOTOR_BRINGUP_TEST_RUNNING,
    MOTOR_BRINGUP_TEST_COMPLETE,
    MOTOR_BRINGUP_TEST_ABORTED
} motor_bringup_test_state_t;

void MotorBringupTest_Init(bool reset_locked);
void MotorBringupTest_Task(uint32_t now_ms, bool press_event);

motor_bringup_test_state_t MotorBringupTest_GetState(void);
const char *MotorBringupTest_GetStateText(void);
int16_t MotorBringupTest_GetCommand(void);
int16_t MotorBringupTest_GetCommandA(void);
int16_t MotorBringupTest_GetCommandB(void);
uint32_t MotorBringupTest_GetRunCount(void);
uint32_t MotorBringupTest_GetMotorChannel(void);
const char *MotorBringupTest_GetMotorText(void);

#endif
