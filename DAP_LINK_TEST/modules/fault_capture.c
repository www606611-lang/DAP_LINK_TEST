#include "fault_capture.h"

#include "ti_msp_dl_config.h"

#include <stddef.h>

#define FAULT_CAPTURE_MAGIC   0x464C5431UL
#define FAULT_CAPTURE_VERSION 1UL
#define FAULT_CAPTURE_SRAM_START 0x20200000UL
#define FAULT_CAPTURE_SRAM_END   0x20208000UL

static volatile fault_capture_record_t g_fault_capture_record
    __attribute__((section(".noinit.fault_capture")));
static volatile uint32_t g_fault_capture_stage
    __attribute__((section(".noinit.fault_stage")));

static uint32_t fault_capture_checksum(
    const fault_capture_record_t *record);
static bool fault_capture_stack_is_readable(const uint32_t *stack);
static void fault_capture_store_and_reset(
    uint32_t *stack, uint32_t exc_return, uint32_t reason);
void FaultCapture_HandlerC(
    uint32_t *stack, uint32_t exc_return, uint32_t reason);

bool FaultCapture_Read(fault_capture_record_t *record)
{
    fault_capture_record_t snapshot;
    const volatile fault_capture_record_t *saved = &g_fault_capture_record;

    if (record == NULL) {
        return false;
    }

    snapshot.magic = saved->magic;
    snapshot.version = saved->version;
    snapshot.reason = saved->reason;
    snapshot.active_vector = saved->active_vector;
    snapshot.nmi_iidx = saved->nmi_iidx;
    snapshot.exc_return = saved->exc_return;
    snapshot.fault_sp = saved->fault_sp;
    snapshot.stacked_r0 = saved->stacked_r0;
    snapshot.stacked_r1 = saved->stacked_r1;
    snapshot.stacked_r2 = saved->stacked_r2;
    snapshot.stacked_r3 = saved->stacked_r3;
    snapshot.stacked_r12 = saved->stacked_r12;
    snapshot.stacked_lr = saved->stacked_lr;
    snapshot.stacked_pc = saved->stacked_pc;
    snapshot.stacked_xpsr = saved->stacked_xpsr;
    snapshot.icsr = saved->icsr;
    snapshot.shcsr = saved->shcsr;
    snapshot.msp = saved->msp;
    snapshot.psp = saved->psp;
    snapshot.checksum = saved->checksum;

    if ((snapshot.magic != FAULT_CAPTURE_MAGIC) ||
        (snapshot.version != FAULT_CAPTURE_VERSION) ||
        (snapshot.checksum != fault_capture_checksum(&snapshot))) {
        return false;
    }

    *record = snapshot;
    return true;
}

void FaultCapture_Clear(void)
{
    g_fault_capture_record.magic = 0U;
    g_fault_capture_record.version = 0U;
    g_fault_capture_record.reason = 0U;
    g_fault_capture_record.checksum = 0U;
}

void FaultCapture_SetStage(uint32_t stage)
{
    g_fault_capture_stage = stage;
}

uint32_t FaultCapture_GetStage(void)
{
    return g_fault_capture_stage;
}

void FaultCapture_ClearStage(void)
{
    g_fault_capture_stage = 0U;
}

void FaultCapture_HandlerC(
    uint32_t *stack, uint32_t exc_return, uint32_t reason)
{
    fault_capture_store_and_reset(stack, exc_return, reason);
}

void FaultCapture_NMIHandlerC(uint32_t *stack, uint32_t exc_return)
{
    fault_capture_store_and_reset(stack, exc_return, FAULT_CAPTURE_REASON_NMI);
}

__attribute__((naked)) void HardFault_Handler(void)
{
    __asm volatile(
        "movs r0, #4\n"
        "mov r1, lr\n"
        "tst r0, r1\n"
        "beq 1f\n"
        "mrs r0, psp\n"
        "b 2f\n"
        "1:\n"
        "mrs r0, msp\n"
        "2:\n"
        "mov r1, lr\n"
        "movs r2, #1\n"
        "b FaultCapture_HandlerC\n");
}

__attribute__((naked)) void NMI_Handler(void)
{
    __asm volatile(
        "movs r0, #4\n"
        "mov r1, lr\n"
        "tst r0, r1\n"
        "beq 1f\n"
        "mrs r0, psp\n"
        "b 2f\n"
        "1:\n"
        "mrs r0, msp\n"
        "2:\n"
        "mov r1, lr\n"
        "b FaultCapture_NMIHandlerC\n");
}

static uint32_t fault_capture_checksum(
    const fault_capture_record_t *record)
{
    const uint32_t *words = (const uint32_t *) record;
    uint32_t checksum = 0xA5A55A5AUL;
    uint32_t count =
        (uint32_t) (offsetof(fault_capture_record_t, checksum) /
                    sizeof(uint32_t));
    uint32_t i;

    for (i = 0U; i < count; i++) {
        checksum ^= words[i] + 0x9E3779B9UL + (checksum << 6) +
                    (checksum >> 2);
    }

    return checksum;
}

static void fault_capture_store_and_reset(
    uint32_t *stack, uint32_t exc_return, uint32_t reason)
{
    fault_capture_record_t record;
    bool stack_ok;

    __disable_irq();
    stack_ok = fault_capture_stack_is_readable(stack);

    record.magic = FAULT_CAPTURE_MAGIC;
    record.version = FAULT_CAPTURE_VERSION;
    record.reason = reason;
    record.active_vector =
        (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) >> SCB_ICSR_VECTACTIVE_Pos;
    record.nmi_iidx = 0U;
    if (reason == FAULT_CAPTURE_REASON_NMI) {
        record.nmi_iidx =
            (uint32_t) DL_SYSCTL_getPendingNonMaskableInterrupt();
    }
    record.exc_return = exc_return;
    record.fault_sp = (uint32_t) stack;
    record.stacked_r0 = stack_ok ? stack[0] : 0U;
    record.stacked_r1 = stack_ok ? stack[1] : 0U;
    record.stacked_r2 = stack_ok ? stack[2] : 0U;
    record.stacked_r3 = stack_ok ? stack[3] : 0U;
    record.stacked_r12 = stack_ok ? stack[4] : 0U;
    record.stacked_lr = stack_ok ? stack[5] : 0U;
    record.stacked_pc = stack_ok ? stack[6] : 0U;
    record.stacked_xpsr = stack_ok ? stack[7] : 0U;
    record.icsr = SCB->ICSR;
    record.shcsr = SCB->SHCSR;
    record.msp = __get_MSP();
    record.psp = __get_PSP();
    record.checksum = fault_capture_checksum(&record);

    g_fault_capture_record = record;

    DL_SYSCTL_resetDevice(DL_SYSCTL_RESET_SYSRST);
    while (1) {
    }
}

static bool fault_capture_stack_is_readable(const uint32_t *stack)
{
    uint32_t address = (uint32_t) stack;

    if ((address < FAULT_CAPTURE_SRAM_START) ||
        (address > (FAULT_CAPTURE_SRAM_END - (8U * sizeof(uint32_t)))) ||
        ((address & 0x3U) != 0U)) {
        return false;
    }

    return true;
}
