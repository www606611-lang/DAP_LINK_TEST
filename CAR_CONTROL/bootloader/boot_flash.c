#include "boot_flash.h"

#include "firmware_layout.h"
#include "ti_msp_dl_config.h"

static bool boot_flash_range_allowed(uint32_t address, uint32_t length)
{
    uint32_t end;

    if ((length == 0U) || (address > (UINT32_MAX - length))) {
        return false;
    }
    end = address + length;
    if ((address >= FIRMWARE_APPLICATION_START) &&
        (end <= FIRMWARE_METADATA_ADDRESS)) {
        return true;
    }
    return (address >= FIRMWARE_METADATA_ADDRESS) &&
        (end <= FIRMWARE_FLASH_END);
}

bool BootFlash_EraseSector(uint32_t address)
{
    DL_FLASHCTL_COMMAND_STATUS status;

    if (((address & (DL_FLASHCTL_SECTOR_SIZE - 1U)) != 0U) ||
        !boot_flash_range_allowed(address, DL_FLASHCTL_SECTOR_SIZE)) {
        return false;
    }

    DL_FlashCTL_executeClearStatus(FLASHCTL);
    DL_FlashCTL_unprotectSector(
        FLASHCTL, address, DL_FLASHCTL_REGION_SELECT_MAIN);
    status = DL_FlashCTL_eraseMemoryFromRAM(
        FLASHCTL, address, DL_FLASHCTL_COMMAND_SIZE_SECTOR);
    return status == DL_FLASHCTL_COMMAND_STATUS_PASSED;
}

bool BootFlash_Program(
    uint32_t address, const uint32_t *data, uint32_t length_bytes)
{
    DL_FLASHCTL_COMMAND_STATUS status;

    if ((data == 0) || ((address & 7U) != 0U) ||
        ((length_bytes & 7U) != 0U) ||
        !boot_flash_range_allowed(address, length_bytes)) {
        return false;
    }

    status = DL_FlashCTL_programMemoryBlockingFromRAM64WithECCGenerated(
        FLASHCTL, address, (uint32_t *) data, length_bytes / 4U,
        DL_FLASHCTL_REGION_SELECT_MAIN);
    return status == DL_FLASHCTL_COMMAND_STATUS_PASSED;
}
