#ifndef APP_TUNING_TUNING_WIRE_H
#define APP_TUNING_TUNING_WIRE_H

#include <stdint.h>

void speed_tuning_write_u32(uint32_t value);
void speed_tuning_write_i32(int32_t value);
void speed_tuning_write_float4(float value);
int32_t speed_tuning_round_float(float value);

#endif
