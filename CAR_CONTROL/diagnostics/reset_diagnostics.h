#ifndef DIAGNOSTICS_RESET_DIAGNOSTICS_H
#define DIAGNOSTICS_RESET_DIAGNOSTICS_H

#include <stdbool.h>
#include <stdint.h>

void ResetDiagnostics_Init(void);

uint32_t ResetDiagnostics_GetCause(void);
const char *ResetDiagnostics_GetCauseText(void);
bool ResetDiagnostics_IsSuspicious(void);

#endif
