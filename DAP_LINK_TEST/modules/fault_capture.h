#ifndef MODULES_FAULT_CAPTURE_H
#define MODULES_FAULT_CAPTURE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FAULT_CAPTURE_REASON_NONE = 0,
    FAULT_CAPTURE_REASON_HARDFAULT = 1,
    FAULT_CAPTURE_REASON_NMI = 2
} fault_capture_reason_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t reason;
    uint32_t active_vector;
    uint32_t nmi_iidx;
    uint32_t exc_return;
    uint32_t fault_sp;
    uint32_t stacked_r0;
    uint32_t stacked_r1;
    uint32_t stacked_r2;
    uint32_t stacked_r3;
    uint32_t stacked_r12;
    uint32_t stacked_lr;
    uint32_t stacked_pc;
    uint32_t stacked_xpsr;
    uint32_t icsr;
    uint32_t shcsr;
    uint32_t msp;
    uint32_t psp;
    uint32_t checksum;
} fault_capture_record_t;

bool FaultCapture_Read(fault_capture_record_t *record);
void FaultCapture_Clear(void);
void FaultCapture_NMIHandlerC(uint32_t *stack, uint32_t exc_return);
void FaultCapture_SetStage(uint32_t stage);
uint32_t FaultCapture_GetStage(void);
void FaultCapture_ClearStage(void);

#ifdef __cplusplus
}
#endif

#endif
