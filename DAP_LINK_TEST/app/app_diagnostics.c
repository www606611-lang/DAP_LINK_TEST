#include "app_diagnostics.h"

#include "fault_capture.h"
#include "ti_msp_dl_config.h"
#include "uart0_dma.h"

#include <stdbool.h>
#include <stdint.h>

static const char *app_diagnostics_reset_cause_to_string(
    DL_SYSCTL_RESET_CAUSE cause);
static const char *app_diagnostics_fault_reason_to_string(uint32_t reason);
static const char *app_diagnostics_nmi_source_to_string(uint32_t nmi_iidx);
static void app_diagnostics_append_char(
    char *line, uint16_t *length, uint16_t max_length, char value);
static void app_diagnostics_append_int32(
    char *line, uint16_t *length, uint16_t max_length, int32_t value);
static void app_diagnostics_append_hex32(
    char *line, uint16_t *length, uint16_t max_length, uint32_t value);

void AppDiagnostics_SetStage(app_diagnostics_stage_t stage)
{
    FaultCapture_SetStage((uint32_t) stage);
}

void AppDiagnostics_ClearStage(void)
{
    FaultCapture_ClearStage();
}

void AppDiagnostics_ReportResetCause(void)
{
    char line[128];
    uint16_t length = 0U;
    const char prefix[] = "RST:";
    fault_capture_record_t fault;
    bool has_fault;
    DL_SYSCTL_RESET_CAUSE cause = DL_SYSCTL_getResetCause();
    const char *text = app_diagnostics_reset_cause_to_string(cause);
    uint32_t stage = FaultCapture_GetStage();
    uint16_t i;

    for (i = 0U; (i < (uint16_t) (sizeof(prefix) - 1U)) &&
        (length < sizeof(line)); i++) {
        line[length++] = prefix[i];
    }

    while ((*text != '\0') && (length < sizeof(line))) {
        line[length++] = *text++;
    }

    has_fault = FaultCapture_Read(&fault);
    if (has_fault) {
        const char *reason =
            app_diagnostics_fault_reason_to_string(fault.reason);

        app_diagnostics_append_char(line, &length, sizeof(line), ' ');
        app_diagnostics_append_char(line, &length, sizeof(line), 'F');
        app_diagnostics_append_char(line, &length, sizeof(line), ':');
        while ((*reason != '\0') && (length < sizeof(line))) {
            line[length++] = *reason++;
        }
        if (fault.reason == FAULT_CAPTURE_REASON_NMI) {
            const char *nmi =
                app_diagnostics_nmi_source_to_string(fault.nmi_iidx);

            app_diagnostics_append_char(line, &length, sizeof(line), ' ');
            app_diagnostics_append_char(line, &length, sizeof(line), 'N');
            app_diagnostics_append_char(line, &length, sizeof(line), ':');
            while ((*nmi != '\0') && (length < sizeof(line))) {
                line[length++] = *nmi++;
            }
        }
        app_diagnostics_append_char(line, &length, sizeof(line), ' ');
        app_diagnostics_append_char(line, &length, sizeof(line), 'V');
        app_diagnostics_append_char(line, &length, sizeof(line), ':');
        app_diagnostics_append_int32(line, &length, sizeof(line),
            (int32_t) fault.active_vector);
        app_diagnostics_append_char(line, &length, sizeof(line), ' ');
        app_diagnostics_append_char(line, &length, sizeof(line), 'P');
        app_diagnostics_append_char(line, &length, sizeof(line), 'C');
        app_diagnostics_append_char(line, &length, sizeof(line), ':');
        app_diagnostics_append_hex32(line, &length, sizeof(line),
            fault.stacked_pc);
        app_diagnostics_append_char(line, &length, sizeof(line), ' ');
        app_diagnostics_append_char(line, &length, sizeof(line), 'L');
        app_diagnostics_append_char(line, &length, sizeof(line), 'R');
        app_diagnostics_append_char(line, &length, sizeof(line), ':');
        app_diagnostics_append_hex32(line, &length, sizeof(line),
            fault.stacked_lr);
        app_diagnostics_append_char(line, &length, sizeof(line), ' ');
        app_diagnostics_append_char(line, &length, sizeof(line), 'S');
        app_diagnostics_append_char(line, &length, sizeof(line), 'P');
        app_diagnostics_append_char(line, &length, sizeof(line), ':');
        app_diagnostics_append_hex32(line, &length, sizeof(line),
            fault.fault_sp);
        FaultCapture_Clear();
    }
    if (stage != 0U) {
        app_diagnostics_append_char(line, &length, sizeof(line), ' ');
        app_diagnostics_append_char(line, &length, sizeof(line), 'S');
        app_diagnostics_append_char(line, &length, sizeof(line), ':');
        app_diagnostics_append_int32(line, &length, sizeof(line),
            (int32_t) stage);
    }
    FaultCapture_ClearStage();

    if ((length + 2U) <= sizeof(line)) {
        line[length++] = '\r';
        line[length++] = '\n';
    }

    if (length > 0U) {
        (void) uart0_dma_send((const uint8_t *) line, length);
    }
}

static const char *app_diagnostics_fault_reason_to_string(uint32_t reason)
{
    switch (reason) {
        case FAULT_CAPTURE_REASON_HARDFAULT:
            return "HF";
        case FAULT_CAPTURE_REASON_NMI:
            return "NMI";
        default:
            return "UNK";
    }
}

static const char *app_diagnostics_nmi_source_to_string(uint32_t nmi_iidx)
{
    switch (nmi_iidx) {
        case DL_SYSCTL_NMI_IIDX_SRAM_DED:
            return "SRAM_DED";
        case DL_SYSCTL_NMI_IIDX_FLASH_DED:
            return "FLASH_DED";
        case DL_SYSCTL_NMI_IIDX_LFCLK_FAIL:
            return "LFCLK_FAIL";
        case DL_SYSCTL_NMI_IIDX_WWDT1_FAULT:
            return "WWDT1";
        case DL_SYSCTL_NMI_IIDX_WWDT0_FAULT:
            return "WWDT0";
        case DL_SYSCTL_NMI_IIDX_BORLVL:
            return "BORLVL";
        case DL_SYSCTL_NMI_IIDX_NO_INT:
            return "NO_INT";
        default:
            return "UNK";
    }
}

static const char *app_diagnostics_reset_cause_to_string(
    DL_SYSCTL_RESET_CAUSE cause)
{
    switch (cause) {
        case DL_SYSCTL_RESET_CAUSE_NO_RESET:
            return "NO_RESET";
        case DL_SYSCTL_RESET_CAUSE_POR_HW_FAILURE:
            return "POR_HW_FAILURE";
        case DL_SYSCTL_RESET_CAUSE_POR_EXTERNAL_NRST:
            return "POR_EXTERNAL_NRST";
        case DL_SYSCTL_RESET_CAUSE_POR_SW_TRIGGERED:
            return "POR_SW_TRIGGERED";
        case DL_SYSCTL_RESET_CAUSE_BOR_SUPPLY_FAILURE:
            return "BOR_SUPPLY_FAILURE";
        case DL_SYSCTL_RESET_CAUSE_BOR_WAKE_FROM_SHUTDOWN:
            return "BOR_WAKE_FROM_SHUTDOWN";
        case DL_SYSCTL_RESET_CAUSE_BOOTRST_NON_PMU_PARITY_FAULT:
            return "BOOTRST_NON_PMU_PARITY_FAULT";
        case DL_SYSCTL_RESET_CAUSE_BOOTRST_CLOCK_FAULT:
            return "BOOTRST_CLOCK_FAULT";
        case DL_SYSCTL_RESET_CAUSE_BOOTRST_SW_TRIGGERED:
            return "BOOTRST_SW_TRIGGERED";
        case DL_SYSCTL_RESET_CAUSE_BOOTRST_EXTERNAL_NRST:
            return "BOOTRST_EXTERNAL_NRST";
        case DL_SYSCTL_RESET_CAUSE_SYSRST_BSL_EXIT:
            return "SYSRST_BSL_EXIT";
        case DL_SYSCTL_RESET_CAUSE_SYSRST_BSL_ENTRY:
            return "SYSRST_BSL_ENTRY";
        case DL_SYSCTL_RESET_CAUSE_SYSRST_WWDT0_VIOLATION:
            return "SYSRST_WWDT0_VIOLATION";
        case DL_SYSCTL_RESET_CAUSE_SYSRST_WWDT1_VIOLATION:
            return "SYSRST_WWDT1_VIOLATION";
        case DL_SYSCTL_RESET_CAUSE_SYSRST_FLASH_ECC_ERROR:
            return "SYSRST_FLASH_ECC_ERROR";
        case DL_SYSCTL_RESET_CAUSE_SYSRST_CPU_LOCKUP_VIOLATION:
            return "SYSRST_CPU_LOCKUP_VIOLATION";
        case DL_SYSCTL_RESET_CAUSE_SYSRST_DEBUG_TRIGGERED:
            return "SYSRST_DEBUG_TRIGGERED";
        case DL_SYSCTL_RESET_CAUSE_SYSRST_SW_TRIGGERED:
            return "SYSRST_SW_TRIGGERED";
        case DL_SYSCTL_RESET_CAUSE_CPURST_DEBUG_TRIGGERED:
            return "CPURST_DEBUG_TRIGGERED";
        case DL_SYSCTL_RESET_CAUSE_CPURST_SW_TRIGGERED:
            return "CPURST_SW_TRIGGERED";
        default:
            return "UNKNOWN";
    }
}

static void app_diagnostics_append_char(
    char *line, uint16_t *length, uint16_t max_length, char value)
{
    if ((line == 0) || (length == 0) || (*length >= max_length)) {
        return;
    }

    line[*length] = value;
    (*length)++;
}

static void app_diagnostics_append_int32(
    char *line, uint16_t *length, uint16_t max_length, int32_t value)
{
    char digits[11];
    uint8_t digit_count = 0U;
    uint32_t magnitude;

    if ((line == 0) || (length == 0)) {
        return;
    }

    if (value < 0) {
        app_diagnostics_append_char(line, length, max_length, '-');
        magnitude = (uint32_t) (-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t) value;
    }

    do {
        digits[digit_count++] = (char) ('0' + (magnitude % 10U));
        magnitude /= 10U;
    } while ((magnitude > 0U) && (digit_count < sizeof(digits)));

    while (digit_count > 0U) {
        app_diagnostics_append_char(line, length, max_length,
            digits[--digit_count]);
    }
}

static void app_diagnostics_append_hex32(
    char *line, uint16_t *length, uint16_t max_length, uint32_t value)
{
    int8_t shift;

    app_diagnostics_append_char(line, length, max_length, '0');
    app_diagnostics_append_char(line, length, max_length, 'x');

    for (shift = 28; shift >= 0; shift -= 4) {
        uint8_t digit = (uint8_t) ((value >> shift) & 0x0FU);
        app_diagnostics_append_char(line, length, max_length,
            (char) ((digit < 10U) ? ('0' + digit) :
                                  ('A' + (digit - 10U))));
    }
}
