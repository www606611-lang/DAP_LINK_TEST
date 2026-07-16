#include "crc32.h"

uint32_t Crc32_Begin(void)
{
    return 0xFFFFFFFFUL;
}

uint32_t Crc32_Update(uint32_t crc, const uint8_t *data, uint32_t length)
{
    uint32_t i;

    if (data == 0) {
        return crc;
    }
    while (length-- > 0U) {
        crc ^= *data++;
        for (i = 0U; i < 8U; i++) {
            crc = (crc >> 1U) ^ ((crc & 1U) ? 0xEDB88320UL : 0U);
        }
    }
    return crc;
}

uint32_t Crc32_End(uint32_t crc)
{
    return crc ^ 0xFFFFFFFFUL;
}

uint32_t Crc32_Calculate(const uint8_t *data, uint32_t length)
{
    return Crc32_End(Crc32_Update(Crc32_Begin(), data, length));
}
