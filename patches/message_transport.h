#pragma once
#include <stdint.h>

#define CFW_MESSAGE_SID 0xf0u
#define CFW_MESSAGE_LEFT 1u
#define CFW_MESSAGE_RIGHT 2u
#define CFW_MESSAGE_BOTH (CFW_MESSAGE_LEFT | CFW_MESSAGE_RIGHT)
#define CFW_MESSAGE_ACK 1u
#define CFW_MESSAGE_NACK 3u
#define CFW_MESSAGE_COMPRESSED 4u
#define CFW_MESSAGE_RESET_CONTEXT 8u
#define CFW_MESSAGE_MAX 65535u
#define CFW_MESSAGE_RESET 0x80u
#define CFW_MESSAGE_END 0x40u
#define CFW_ACK_HISTORY 3u
#define CFW_ACK_ENTRY_SIZE 7u /* stream, ordinal LE16, size LE16, CRC LE16 */

/* One ordered byte stream per BLE ingress lens. Payload is owned until the
 * handler returns, or a packet reset/error discards it. A record contains
 * clear flags, uint16 wire length, uint16 decoded CRC, then the body.
 * Packet RESET/END affect reconstruction only; RESET_CONTEXT affects zlib. */
typedef struct {
    uint8_t *buffer;
    void *inflater;
    uint16_t size, used, message_id, checksum;
    uint8_t length_bytes, active, next_sequence, stream_id, options;
    uint8_t flags, context_valid;
    uint8_t ack_count, ack_capacity, packet_capacity;
    uint8_t ack_history[CFW_ACK_HISTORY][CFW_ACK_ENTRY_SIZE];
} cfw_message_stream;
int cfw_message_received(const uint8_t *data, uint16_t size, uint16_t checksum);
uint32_t cfw_receive_packet(uint8_t pipe, const uint8_t *packet, uint16_t length);
uint32_t cfw_message_bridge_received(uint32_t app_id, const uint8_t *data,
                                     uint32_t length, uint16_t event);
