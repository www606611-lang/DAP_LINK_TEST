#ifndef DRIVERS_UTILITY_CRC32_H
#define DRIVERS_UTILITY_CRC32_H

#include <stdint.h>

uint32_t Crc32_Begin(void);
uint32_t Crc32_Update(uint32_t crc, const uint8_t *data, uint32_t length);
uint32_t Crc32_End(uint32_t crc);
uint32_t Crc32_Calculate(const uint8_t *data, uint32_t length);

#endif
