#ifndef DRIVERS_DEVICE_CHASSIS_RADIO_LINK_H
#define DRIVERS_DEVICE_CHASSIS_RADIO_LINK_H

#include <stdbool.h>
#include <stdint.h>

#define CHASSIS_RADIO_OFFLINE_TIMEOUT_MS 1000U
#define CHASSIS_RADIO_STATUS_HIGH_Z      (1U << 0)

typedef struct {
    bool online;
    bool esp32_online;
    bool k230_online;
    uint8_t last_role;
    uint8_t status_flags;
    uint32_t last_frame_ms;
    uint32_t rx_frame_count;
    uint32_t tx_frame_count;
    uint32_t duplicate_count;
    uint32_t out_of_order_count;
    uint32_t shadow_command_count;
    uint32_t unsupported_count;
    uint32_t timeout_count;
    uint32_t crc_error_count;
    uint32_t length_error_count;
    uint32_t version_error_count;
    uint32_t resync_count;
    uint32_t received_byte_count;
    uint32_t rx_overflow_count;
    uint32_t transmitted_byte_count;
    uint32_t tx_rejected_count;
} chassis_radio_snapshot_t;

void ChassisRadioLink_Init(uint32_t now_ms);
void ChassisRadioLink_SetStatusFlags(uint8_t status_flags);
void ChassisRadioLink_Task(uint32_t now_ms);
bool ChassisRadioLink_GetSnapshot(chassis_radio_snapshot_t *snapshot);

#endif
