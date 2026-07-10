#ifndef COMMON_DELAY_H
#define COMMON_DELAY_H

#include <stdint.h>

void delay_cpu_cycles(uint32_t cycles);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
uint32_t delay_get_ms(void);

#endif
