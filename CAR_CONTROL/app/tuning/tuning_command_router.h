#ifndef APP_TUNING_TUNING_COMMAND_ROUTER_H
#define APP_TUNING_TUNING_COMMAND_ROUTER_H

#include <stdint.h>

void TuningCommandRouter_ProcessLine(char *line, uint32_t now_ms);

#endif
