#ifndef BOOTLOADER_BOOT_FLASH_H
#define BOOTLOADER_BOOT_FLASH_H

#include <stdbool.h>
#include <stdint.h>

bool BootFlash_EraseSector(uint32_t address);
bool BootFlash_Program(
    uint32_t address, const uint32_t *data, uint32_t length_bytes);

#endif
