#pragma once
#include <stdint.h>

#define CFW_MESSAGE_SID 0xf0u
#define CFW_MESSAGE_LEFT 1u
#define CFW_MESSAGE_RIGHT 2u
#define CFW_MESSAGE_BOTH (CFW_MESSAGE_LEFT | CFW_MESSAGE_RIGHT)
#define CFW_MESSAGE_ACK 1u
int cfw_message_received(const uint8_t *data, uint16_t size, uint16_t checksum);
uint32_t cfw_receive_packet(uint8_t pipe, const uint8_t *packet, uint16_t length);
uint32_t cfw_message_bridge_received(uint32_t app_id, const uint8_t *data,
                                     uint32_t length, uint16_t event);
