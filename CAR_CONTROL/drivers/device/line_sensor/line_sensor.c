#include "line_sensor.h"

#include "i2c1_polling.h"

#include <stddef.h>

#define LINE_SENSOR_I2C_ADDRESS7            0x12U
#define LINE_SENSOR_CONTROL_REGISTER        0x01U
#define LINE_SENSOR_DATA_REGISTER           0x30U
#define LINE_SENSOR_SAMPLE_INTERVAL_MS        10U
#define LINE_SENSOR_BOOT_WAIT_MS             200U
#define LINE_SENSOR_CALIBRATION_HOLD_MS       200U
#define LINE_SENSOR_CALIBRATION_SETTLE_MS     200U
#define LINE_SENSOR_ERROR_RETRY_MS            100U

static const int16_t g_channel_weights[LINE_SENSOR_CHANNEL_COUNT] = {
    -35, -25, -15, -5, 5, 15, 25, 35
};

static line_sensor_snapshot_t g_snapshot;
static uint32_t g_deadline_ms;
static uint32_t g_next_sample_ms;
static bool g_retry_calibration;

static bool line_sensor_write_calibration(bool enabled);
static bool line_sensor_read_sample(uint32_t now_ms);
static void line_sensor_enter_error(
    uint32_t now_ms, bool retry_calibration);
static bool line_sensor_deadline_reached(
    uint32_t now_ms, uint32_t deadline_ms);

void LineSensor_Init(uint32_t now_ms)
{
    uint8_t i;

    I2C1Polling_Init();
    for (i = 0U; i < LINE_SENSOR_CHANNEL_COUNT; i++) {
        g_snapshot.channel[i] = false;
    }
    g_snapshot.raw = 0xFFU;
    g_snapshot.active_mask = 0U;
    g_snapshot.active_count = 0U;
    g_snapshot.line_error = 0;
    g_snapshot.sample_count = 0U;
    g_snapshot.read_error_count = 0U;
    g_snapshot.calibration_count = 0U;
    g_snapshot.last_sample_ms = now_ms;
    g_snapshot.state = LINE_SENSOR_STATE_BOOT_WAIT;
    g_snapshot.last_result = LINE_SENSOR_OK;
    g_snapshot.line_seen = false;
    g_snapshot.ready = false;
    g_deadline_ms = now_ms + LINE_SENSOR_BOOT_WAIT_MS;
    g_next_sample_ms = now_ms;
    g_retry_calibration = true;
}

void LineSensor_Task(uint32_t now_ms)
{
    switch (g_snapshot.state) {
        case LINE_SENSOR_STATE_UNINITIALIZED:
            return;

        case LINE_SENSOR_STATE_BOOT_WAIT:
            if (!line_sensor_deadline_reached(now_ms, g_deadline_ms)) {
                return;
            }
            if (!line_sensor_write_calibration(true)) {
                line_sensor_enter_error(now_ms, true);
                return;
            }
            g_snapshot.state = LINE_SENSOR_STATE_CALIBRATION_ON;
            g_deadline_ms = now_ms + LINE_SENSOR_CALIBRATION_HOLD_MS;
            return;

        case LINE_SENSOR_STATE_CALIBRATION_ON:
            if (!line_sensor_deadline_reached(now_ms, g_deadline_ms)) {
                return;
            }
            if (!line_sensor_write_calibration(false)) {
                line_sensor_enter_error(now_ms, true);
                return;
            }
            g_snapshot.state = LINE_SENSOR_STATE_CALIBRATION_OFF_WAIT;
            g_deadline_ms = now_ms + LINE_SENSOR_CALIBRATION_SETTLE_MS;
            return;

        case LINE_SENSOR_STATE_CALIBRATION_OFF_WAIT:
            if (!line_sensor_deadline_reached(now_ms, g_deadline_ms)) {
                return;
            }
            g_snapshot.calibration_count++;
            g_snapshot.state = LINE_SENSOR_STATE_READY;
            g_snapshot.last_result = LINE_SENSOR_OK;
            g_next_sample_ms = now_ms;
            return;

        case LINE_SENSOR_STATE_READY:
            if (!line_sensor_deadline_reached(now_ms, g_next_sample_ms)) {
                return;
            }
            g_next_sample_ms = now_ms + LINE_SENSOR_SAMPLE_INTERVAL_MS;
            if (!line_sensor_read_sample(now_ms)) {
                line_sensor_enter_error(now_ms, false);
            }
            return;

        case LINE_SENSOR_STATE_ERROR:
            if (!line_sensor_deadline_reached(now_ms, g_deadline_ms)) {
                return;
            }
            if (g_retry_calibration) {
                g_snapshot.state = LINE_SENSOR_STATE_BOOT_WAIT;
                g_deadline_ms = now_ms;
            } else if (line_sensor_read_sample(now_ms)) {
                g_snapshot.state = LINE_SENSOR_STATE_READY;
                g_next_sample_ms = now_ms +
                    LINE_SENSOR_SAMPLE_INTERVAL_MS;
            } else {
                g_deadline_ms = now_ms + LINE_SENSOR_ERROR_RETRY_MS;
            }
            return;

        default:
            line_sensor_enter_error(now_ms, true);
            return;
    }
}

line_sensor_result_t LineSensor_RequestCalibration(uint32_t now_ms)
{
    if ((g_snapshot.state == LINE_SENSOR_STATE_BOOT_WAIT) ||
        (g_snapshot.state == LINE_SENSOR_STATE_CALIBRATION_ON) ||
        (g_snapshot.state == LINE_SENSOR_STATE_CALIBRATION_OFF_WAIT)) {
        g_snapshot.last_result = LINE_SENSOR_BUSY;
        return g_snapshot.last_result;
    }
    g_snapshot.ready = false;
    g_snapshot.line_seen = false;
    g_snapshot.state = LINE_SENSOR_STATE_BOOT_WAIT;
    g_snapshot.last_result = LINE_SENSOR_OK;
    g_deadline_ms = now_ms;
    g_retry_calibration = true;
    return LINE_SENSOR_OK;
}

bool LineSensor_GetSnapshot(line_sensor_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        g_snapshot.last_result = LINE_SENSOR_BAD_ARGUMENT;
        return false;
    }
    *snapshot = g_snapshot;
    return true;
}

const char *LineSensor_GetStateText(void)
{
    switch (g_snapshot.state) {
        case LINE_SENSOR_STATE_UNINITIALIZED:
            return "OFF";
        case LINE_SENSOR_STATE_BOOT_WAIT:
        case LINE_SENSOR_STATE_CALIBRATION_ON:
        case LINE_SENSOR_STATE_CALIBRATION_OFF_WAIT:
            return "CAL";
        case LINE_SENSOR_STATE_READY:
            return "READY";
        case LINE_SENSOR_STATE_ERROR:
            return "ERROR";
        default:
            return "UNKNOWN";
    }
}

static bool line_sensor_write_calibration(bool enabled)
{
    uint8_t packet[2];

    packet[0] = LINE_SENSOR_CONTROL_REGISTER;
    packet[1] = enabled ? 1U : 0U;
    if (I2C1Polling_Write(LINE_SENSOR_I2C_ADDRESS7,
            packet, sizeof(packet)) != I2C1_POLLING_OK) {
        g_snapshot.read_error_count++;
        g_snapshot.last_result = LINE_SENSOR_I2C_ERROR;
        return false;
    }
    g_snapshot.last_result = LINE_SENSOR_OK;
    return true;
}

static bool line_sensor_read_sample(uint32_t now_ms)
{
    uint8_t raw;
    uint8_t i;
    int32_t weighted_sum = 0;

    if (I2C1Polling_ReadRegister(LINE_SENSOR_I2C_ADDRESS7,
            LINE_SENSOR_DATA_REGISTER, &raw, 1U) != I2C1_POLLING_OK) {
        g_snapshot.read_error_count++;
        g_snapshot.last_result = LINE_SENSOR_I2C_ERROR;
        g_snapshot.ready = false;
        g_snapshot.line_seen = false;
        return false;
    }

    g_snapshot.raw = raw;
    g_snapshot.active_mask = 0U;
    g_snapshot.active_count = 0U;
    for (i = 0U; i < LINE_SENSOR_CHANNEL_COUNT; i++) {
        uint8_t bit = (uint8_t) (0x80U >> i);
        bool active = ((raw & bit) == 0U);

        g_snapshot.channel[i] = active;
        if (active) {
            g_snapshot.active_mask |= bit;
            g_snapshot.active_count++;
            weighted_sum += g_channel_weights[i];
        }
    }
    g_snapshot.line_seen = (g_snapshot.active_count > 0U);
    if (g_snapshot.line_seen) {
        g_snapshot.line_error = (int16_t) (weighted_sum /
            (int32_t) g_snapshot.active_count);
    }
    g_snapshot.sample_count++;
    g_snapshot.last_sample_ms = now_ms;
    g_snapshot.last_result = LINE_SENSOR_OK;
    g_snapshot.ready = true;
    g_retry_calibration = false;
    return true;
}

static void line_sensor_enter_error(
    uint32_t now_ms, bool retry_calibration)
{
    g_snapshot.state = LINE_SENSOR_STATE_ERROR;
    g_snapshot.ready = false;
    g_snapshot.line_seen = false;
    g_snapshot.last_result = LINE_SENSOR_I2C_ERROR;
    g_deadline_ms = now_ms + LINE_SENSOR_ERROR_RETRY_MS;
    g_retry_calibration = retry_calibration;
}

static bool line_sensor_deadline_reached(
    uint32_t now_ms, uint32_t deadline_ms)
{
    return ((int32_t) (now_ms - deadline_ms) >= 0);
}
