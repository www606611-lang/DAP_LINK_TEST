#ifndef PLATFORM_MSPM0G3507_FIRMWARE_LAYOUT_H
#define PLATFORM_MSPM0G3507_FIRMWARE_LAYOUT_H

#include <stdint.h>

#define FIRMWARE_FLASH_START             0x00000000UL
#define FIRMWARE_FLASH_END               0x00020000UL
#define FIRMWARE_BOOTLOADER_START        0x00000000UL
#define FIRMWARE_BOOTLOADER_SIZE         0x00003000UL
#define FIRMWARE_APPLICATION_START       0x00003000UL
#define FIRMWARE_METADATA_ADDRESS        0x0001FC00UL
#define FIRMWARE_METADATA_SIZE           0x00000400UL
#define FIRMWARE_APPLICATION_MAX_SIZE    \
    (FIRMWARE_METADATA_ADDRESS - FIRMWARE_APPLICATION_START)

#define FIRMWARE_SRAM_START              0x20200000UL
#define FIRMWARE_SRAM_END                0x20208000UL
#define FIRMWARE_MAILBOX_ADDRESS         0x20207FE0UL
#define FIRMWARE_MAILBOX_MAGIC           0x424F4F54UL
#define FIRMWARE_MAILBOX_MAGIC_INVERSE   (~FIRMWARE_MAILBOX_MAGIC)

#define FIRMWARE_METADATA_BEGIN_MAGIC    0x46575550UL
#define FIRMWARE_METADATA_COMPLETE_MAGIC 0x434F4D50UL
#define FIRMWARE_METADATA_VERSION        1UL

typedef struct {
    uint32_t magic;
    uint32_t magic_inverse;
    uint32_t reserved[6];
} firmware_mailbox_t;

typedef struct {
    uint32_t begin_magic;
    uint32_t version;
    uint32_t image_size;
    uint32_t image_crc32;
    uint32_t complete_magic;
    uint32_t complete_inverse;
} firmware_metadata_t;

#endif
