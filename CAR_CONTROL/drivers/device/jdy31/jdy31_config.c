#include "jdy31_config.h"

#include "bluetooth_uart.h"

#include <stddef.h>
#include <string.h>

#define JDY31_START_DELAY_MS       1200U
#define JDY31_DISC_WAIT_MS          700U
#define JDY31_REPLY_TIMEOUT_MS      700U
#define JDY31_VERIFY_DELAY_MS       900U
#define JDY31_MAX_QUERY_ATTEMPTS      3U

static jdy31_config_snapshot_t g_snapshot;
static uint32_t g_deadline_ms;
static char g_response[96];

static bool jdy31_deadline_reached(uint32_t now_ms);
static void jdy31_set_state(
    jdy31_config_state_t state, uint32_t deadline_ms);
static void jdy31_send(const char *command, bool crlf);
static void jdy31_send_probe(jdy31_config_state_t state, uint32_t now_ms);
static void jdy31_send_baud_query(
    jdy31_config_state_t state, uint32_t now_ms);
static bool jdy31_response_contains(const char *text);
static int32_t jdy31_parse_baud_code(void);
static void jdy31_save_response(const char *response);
static void jdy31_fail(void);

void JDY31_ConfigInit(uint32_t now_ms, bool configure_on_boot)
{
    memset(&g_snapshot, 0, sizeof(g_snapshot));
    g_snapshot.exclusive = configure_on_boot;
    g_snapshot.reported_baud_code = -1;
    g_snapshot.uart_baud = BluetoothUart_GetBaudRate();
    if (configure_on_boot) {
        jdy31_set_state(JDY31_CONFIG_WAIT_START,
            now_ms + JDY31_START_DELAY_MS);
    } else {
        jdy31_set_state(JDY31_CONFIG_DISABLED, 0U);
    }
}

void JDY31_ConfigTask(uint32_t now_ms)
{
    bluetooth_uart_line_result_t line_result;
    bool have_response = false;

    if (!g_snapshot.exclusive ||
        (g_snapshot.state == JDY31_CONFIG_POWER_CYCLE) ||
        (g_snapshot.state == JDY31_CONFIG_SUCCESS) ||
        (g_snapshot.state == JDY31_CONFIG_FAILED)) {
        return;
    }

    line_result = BluetoothUart_ReadLine(g_response, sizeof(g_response));
    if (line_result == BLUETOOTH_UART_LINE_READY) {
        jdy31_save_response(g_response);
        have_response = true;
    }

    switch (g_snapshot.state) {
    case JDY31_CONFIG_WAIT_START:
        if (jdy31_deadline_reached(now_ms)) {
            jdy31_send("AT+DISC", true);
            jdy31_set_state(JDY31_CONFIG_WAIT_DISC,
                now_ms + JDY31_DISC_WAIT_MS);
        }
        break;

    case JDY31_CONFIG_WAIT_DISC:
        if (jdy31_deadline_reached(now_ms)) {
            jdy31_send_probe(JDY31_CONFIG_PROBE_9600, now_ms);
        }
        break;

    case JDY31_CONFIG_PROBE_9600:
    case JDY31_CONFIG_PROBE_115200:
        if (have_response && jdy31_response_contains("VERSION")) {
            jdy31_send_baud_query(JDY31_CONFIG_QUERY_BAUD, now_ms);
        } else if (jdy31_deadline_reached(now_ms)) {
            if (g_snapshot.command_attempts < JDY31_MAX_QUERY_ATTEMPTS) {
                jdy31_send_probe(g_snapshot.state, now_ms);
            } else if (g_snapshot.state == JDY31_CONFIG_PROBE_9600) {
                if (!BluetoothUart_SetBaudRate(115200U)) {
                    jdy31_fail();
                    break;
                }
                g_snapshot.uart_baud = BluetoothUart_GetBaudRate();
                jdy31_send_probe(JDY31_CONFIG_PROBE_115200, now_ms);
            } else {
                jdy31_fail();
            }
        }
        break;

    case JDY31_CONFIG_QUERY_BAUD:
        if (have_response && jdy31_response_contains("BAUD")) {
            g_snapshot.reported_baud_code = jdy31_parse_baud_code();
            if ((g_snapshot.reported_baud_code == 8) &&
                (BluetoothUart_GetBaudRate() == 115200U)) {
                g_snapshot.success = true;
                jdy31_set_state(JDY31_CONFIG_SUCCESS, 0U);
            } else {
                g_snapshot.command_attempts = 1U;
                jdy31_send("AT+BAUD8", true);
                jdy31_set_state(JDY31_CONFIG_SET_BAUD,
                    now_ms + JDY31_REPLY_TIMEOUT_MS);
            }
        } else if (jdy31_deadline_reached(now_ms)) {
            if (g_snapshot.command_attempts < JDY31_MAX_QUERY_ATTEMPTS) {
                jdy31_send_baud_query(JDY31_CONFIG_QUERY_BAUD, now_ms);
            } else {
                jdy31_fail();
            }
        }
        break;

    case JDY31_CONFIG_SET_BAUD:
        if (have_response &&
            (jdy31_response_contains("OK") ||
             jdy31_response_contains("BAUD"))) {
            jdy31_set_state(JDY31_CONFIG_POWER_CYCLE, 0U);
        } else if (jdy31_deadline_reached(now_ms)) {
            if (!BluetoothUart_SetBaudRate(115200U)) {
                jdy31_fail();
                break;
            }
            g_snapshot.uart_baud = BluetoothUart_GetBaudRate();
            jdy31_set_state(JDY31_CONFIG_VERIFY_DELAY,
                now_ms + JDY31_VERIFY_DELAY_MS);
        }
        break;

    case JDY31_CONFIG_VERIFY_DELAY:
        if (jdy31_deadline_reached(now_ms)) {
            jdy31_send_baud_query(JDY31_CONFIG_VERIFY_BAUD, now_ms);
        }
        break;

    case JDY31_CONFIG_VERIFY_BAUD:
        if (have_response && jdy31_response_contains("BAUD")) {
            g_snapshot.reported_baud_code = jdy31_parse_baud_code();
            if (g_snapshot.reported_baud_code == 8) {
                g_snapshot.success = true;
                jdy31_set_state(JDY31_CONFIG_SUCCESS, 0U);
            } else {
                jdy31_fail();
            }
        } else if (jdy31_deadline_reached(now_ms)) {
            if (g_snapshot.command_attempts < JDY31_MAX_QUERY_ATTEMPTS) {
                jdy31_send_baud_query(
                    JDY31_CONFIG_VERIFY_BAUD, now_ms);
            } else {
                jdy31_fail();
            }
        }
        break;

    default:
        break;
    }
}

bool JDY31_ConfigIsExclusive(void)
{
    return g_snapshot.exclusive;
}

bool JDY31_ConfigGetSnapshot(jdy31_config_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return false;
    }
    *snapshot = g_snapshot;
    return true;
}

const char *JDY31_ConfigGetStateText(void)
{
    switch (g_snapshot.state) {
    case JDY31_CONFIG_DISABLED:      return "DISABLED";
    case JDY31_CONFIG_WAIT_START:    return "START";
    case JDY31_CONFIG_WAIT_DISC:     return "DISC";
    case JDY31_CONFIG_PROBE_9600:    return "PROBE 9600";
    case JDY31_CONFIG_PROBE_115200:  return "PROBE 115200";
    case JDY31_CONFIG_QUERY_BAUD:    return "QUERY BAUD";
    case JDY31_CONFIG_SET_BAUD:      return "SET 115200";
    case JDY31_CONFIG_VERIFY_DELAY:  return "RESTART";
    case JDY31_CONFIG_VERIFY_BAUD:   return "VERIFY";
    case JDY31_CONFIG_POWER_CYCLE:   return "POWER CYCLE";
    case JDY31_CONFIG_SUCCESS:       return "SUCCESS";
    case JDY31_CONFIG_FAILED:        return "FAILED";
    default:                         return "UNKNOWN";
    }
}

static bool jdy31_deadline_reached(uint32_t now_ms)
{
    return ((int32_t) (now_ms - g_deadline_ms) >= 0);
}

static void jdy31_set_state(
    jdy31_config_state_t state, uint32_t deadline_ms)
{
    g_snapshot.state = state;
    g_deadline_ms = deadline_ms;
}

static void jdy31_send(const char *command, bool crlf)
{
    BluetoothUart_WriteText(command);
    BluetoothUart_WriteText(crlf ? "\r\n" : "\r");
}

static void jdy31_send_probe(jdy31_config_state_t state, uint32_t now_ms)
{
    if (g_snapshot.state != state) {
        g_snapshot.command_attempts = 0U;
    }
    g_snapshot.command_attempts++;
    jdy31_send("AT+VERSION", g_snapshot.command_attempts != 2U);
    jdy31_set_state(state, now_ms + JDY31_REPLY_TIMEOUT_MS);
}

static void jdy31_send_baud_query(
    jdy31_config_state_t state, uint32_t now_ms)
{
    if (g_snapshot.state != state) {
        g_snapshot.command_attempts = 0U;
    }
    g_snapshot.command_attempts++;
    jdy31_send("AT+BAUD", g_snapshot.command_attempts != 2U);
    jdy31_set_state(state, now_ms + JDY31_REPLY_TIMEOUT_MS);
}

static bool jdy31_response_contains(const char *text)
{
    return strstr(g_snapshot.last_response, text) != NULL;
}

static int32_t jdy31_parse_baud_code(void)
{
    const char *equals = strrchr(g_snapshot.last_response, '=');

    if ((equals == NULL) || (equals[1] < '0') || (equals[1] > '9')) {
        return -1;
    }
    return (int32_t) (equals[1] - '0');
}

static void jdy31_save_response(const char *response)
{
    size_t length = strlen(response);

    if (length >= sizeof(g_snapshot.last_response)) {
        length = sizeof(g_snapshot.last_response) - 1U;
    }
    memcpy(g_snapshot.last_response, response, length);
    g_snapshot.last_response[length] = '\0';
}

static void jdy31_fail(void)
{
    g_snapshot.success = false;
    g_snapshot.uart_baud = BluetoothUart_GetBaudRate();
    jdy31_set_state(JDY31_CONFIG_FAILED, 0U);
}
