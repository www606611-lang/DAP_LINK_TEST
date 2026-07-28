#include "firmware_update.h"

#include "bluetooth_uart.h"
#include "board_motor_safe.h"
#include "electromagnet.h"
#include "firmware_layout.h"
#include "system_watchdog.h"
#include "ti_msp_dl_config.h"

static bool g_bootloader_pending;

void FirmwareUpdate_AppInit(void)
{
    SCB->VTOR = FIRMWARE_APPLICATION_START;
    __DSB();
    __ISB();
    __enable_irq();
    g_bootloader_pending = false;
}

bool FirmwareUpdate_RequestBootloader(void)
{
    if (g_bootloader_pending || !BoardMotorSafe_IsHighImpedance()) {
        return false;
    }
    Electromagnet_Off();
    BoardMotorSafe_EmergencyStop();
    g_bootloader_pending = true;
    return true;
}

bool FirmwareUpdate_IsPending(void)
{
    return g_bootloader_pending;
}

void FirmwareUpdate_Task(void)
{
    volatile firmware_mailbox_t *mailbox;

    if (!g_bootloader_pending || !BluetoothUart_IsTxIdle()) {
        return;
    }
    Electromagnet_Off();
    SystemWatchdog_PrepareForBootloader();
    mailbox = (volatile firmware_mailbox_t *) FIRMWARE_MAILBOX_ADDRESS;
    mailbox->magic = FIRMWARE_MAILBOX_MAGIC;
    mailbox->magic_inverse = FIRMWARE_MAILBOX_MAGIC_INVERSE;
    __DSB();
    NVIC_SystemReset();
}
