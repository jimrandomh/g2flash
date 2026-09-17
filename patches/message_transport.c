#include "message_transport.h"
#include "memory.h"

#ifndef CFW_STOCK_RECEIVE
#define CFW_STOCK_RECEIVE ((uint32_t (*)(uint8_t, const uint8_t *, uint16_t))0x004cf3e9u)
#define CFW_STOCK_BRIDGE_RECEIVE ((uint32_t (*)(uint32_t, const uint8_t *, uint32_t, uint16_t))0x0045d1a1u)
#define CFW_LENS_SIDE ((uint32_t (*)(void))0x0045cfddu)
/* Both APIs copy the supplied bytes into owned queue storage before returning.
 * SendDataToBoth also delivers a local echo; origin tags below suppress it. */
#define CFW_BRIDGE_SEND ((int (*)(uint16_t, const uint8_t *, uint16_t, void *))0x0046a58du)
#define CFW_BLE_SEND ((int (*)(uint8_t, uint8_t, const uint8_t *, uint16_t))0x0047d72du)
#endif

#define CFW_BRIDGE_REQUEST 1u
#define CFW_BRIDGE_RETURN 2u
#define CFW_ACK_SIZE 9u
#define CFW_ACK_MAX_SIZE (CFW_ACK_SIZE + CFW_ACK_HISTORY * CFW_ACK_ENTRY_SIZE)

#ifndef CFW_STREAM_STATE
static cfw_message_stream *cfw_message_state(uint8_t origin) {
    customCfwContext *ctx = getCustomCfwContext();
    return ctx ? &ctx->message_streams[origin - 1] : 0;
}
#define CFW_STREAM_STATE cfw_message_state
#define CFW_MESSAGE_MALLOC cfw_heap13_malloc
#define CFW_MESSAGE_FREE cfw_heap13_free
#endif

#include "transport_compression.c"

static uint16_t cfw_message_crc(const uint8_t *data, uint16_t size) {
    uint16_t crc = 0xffffu;
    for (uint16_t i = 0; i < size; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (uint16_t)((crc << 1) ^ ((crc & 0x8000u) ? 0x1021u : 0));
    }
    return crc;
}

static uint8_t cfw_message_lens(void) {
    uint32_t side = CFW_LENS_SIDE(); /* Stock identity: 1 = right, 2 = left. */
    return side == 1 ? CFW_MESSAGE_RIGHT : side == 2 ? CFW_MESSAGE_LEFT : 0;
}

static uint32_t cfw_message_validate(const uint8_t *packet, uint16_t length) {
    if (length < 11 || packet[3] != length - 8) return 0xbu;
    if (packet[0] != 0xaa || packet[1] != 0x21 || packet[4] != 1 ||
        packet[5] != 1 || packet[6] != CFW_MESSAGE_SID || packet[7] != 0 ||
        (packet[8] & ~(CFW_MESSAGE_BOTH | CFW_MESSAGE_RESET | CFW_MESSAGE_END))) return 0xau;
    uint16_t expected = (uint16_t)packet[length - 2] |
                        ((uint16_t)packet[length - 1] << 8);
    return cfw_message_crc(packet + 8, length - 10) == expected ? 0 : 0xau;
}

/* Private bridge envelope: kind, ingress-lens bit, body. No borrowed pointer
 * escapes this function. The stock queue wakes its worker on enqueue. */
static int cfw_message_bridge_send(uint8_t kind, uint8_t origin,
                                   const uint8_t *body, uint16_t length) {
    uint8_t envelope[265]; /* 8-byte TPL header + 255 bytes + bridge header. */
    if (length > sizeof(envelope) - 2) return -1;
    envelope[0] = kind;
    envelope[1] = origin;
    memcpy(envelope + 2, body, length);
    return CFW_BRIDGE_SEND(CFW_MESSAGE_SID, envelope, length + 2, 0);
}

static void cfw_message_discard(cfw_message_stream *stream) {
    if (stream->buffer) CFW_MESSAGE_FREE(stream->buffer);
    stream->buffer = 0;
    stream->size = stream->used = 0;
    stream->length_bytes = stream->active = 0;
}

static int cfw_message_reply(cfw_message_stream *stream, uint8_t here,
                             uint8_t origin, uint8_t kind, uint16_t size) {
    uint8_t reply[CFW_ACK_MAX_SIZE] = {kind, stream->stream_id,
        (uint8_t)stream->message_id, (uint8_t)(stream->message_id >> 8), here,
        (uint8_t)size, (uint8_t)(size >> 8),
        (uint8_t)stream->checksum, (uint8_t)(stream->checksum >> 8)};
    uint16_t length = CFW_ACK_SIZE;
    if (kind == CFW_MESSAGE_ACK) {
        /* Explicit successes, newest first; never imply success across a gap.
         * Remember processing even if enqueueing this reply fails, so a later
         * reply can repair that loss without executing the command again. */
        unsigned count = (stream->ack_capacity - CFW_ACK_SIZE) / CFW_ACK_ENTRY_SIZE;
        if (count > stream->ack_count) count = stream->ack_count;
        length += count * CFW_ACK_ENTRY_SIZE;
        memcpy(reply + CFW_ACK_SIZE, stream->ack_history,
               count * CFW_ACK_ENTRY_SIZE);
        for (unsigned i = CFW_ACK_HISTORY - 1; i > 0; --i)
            memcpy(stream->ack_history[i], stream->ack_history[i - 1], CFW_ACK_ENTRY_SIZE);
        memcpy(stream->ack_history[0], reply + 1, 3);
        memcpy(stream->ack_history[0] + 3, reply + 5, 4);
        if (stream->ack_count < CFW_ACK_HISTORY) ++stream->ack_count;
    } else {
        stream->ack_count = 0;
    }
    return here == origin ? CFW_BLE_SEND(1, CFW_MESSAGE_SID, reply, length) :
        cfw_message_bridge_send(CFW_BRIDGE_RETURN, origin, reply, length);
}

/* An incomplete record will never reach the decoded-CRC check. Report the
 * original attempt before discarding it, otherwise the phone waits 3.5 seconds
 * for an ACK while later messages fill its window. Only report once per abort. */
static void cfw_message_abort(cfw_message_stream *stream, uint8_t here, uint8_t origin) {
    if (!stream || !stream->active) return;
    cfw_message_reply(stream, here, origin, CFW_MESSAGE_NACK, 0);
    cfw_inflate_reset(stream);
    stream->context_valid = 0;
    cfw_message_discard(stream);
}

/* ACK identifies the stream and message ordinal, independent of packet splits.
 * Several messages ending in one packet must still have distinct ACKs. */
static uint32_t cfw_message_complete(cfw_message_stream *stream, uint8_t here,
                                     uint8_t origin) {
    uint8_t empty = 0;
    const uint8_t *data = stream->buffer ? stream->buffer : &empty;
    uint8_t *decoded = 0;
    uint32_t size = stream->size;
    int valid = !(stream->flags & ~(CFW_MESSAGE_BOTH | CFW_MESSAGE_COMPRESSED | CFW_MESSAGE_RESET_CONTEXT))
        && (stream->flags & CFW_MESSAGE_BOTH) == stream->options;
    if (stream->flags & CFW_MESSAGE_RESET_CONTEXT) {
        cfw_inflate_reset(stream);
        /* Also bounds history across reconnects, retries and target changes. */
        stream->ack_count = 0;
        stream->ack_capacity = stream->packet_capacity;
        stream->context_valid = 1;
    }
    if (stream->ack_capacity < stream->packet_capacity)
        stream->ack_capacity = stream->packet_capacity;
    valid = valid && stream->context_valid;
    if (stream->flags & CFW_MESSAGE_COMPRESSED) {
        decoded = CFW_MESSAGE_MALLOC(CFW_MESSAGE_MAX + 1u);
        valid = valid && stream->context_valid && decoded && cfw_inflate_message(stream, decoded, &size);
        data = decoded;
    }
    uint16_t crc = valid ? cfw_message_crc(data, (uint16_t)size) : 0;
    valid = valid && crc == stream->checksum;
    if (valid && cfw_message_received(data, (uint16_t)size, crc) != 0) valid = 0;
    if (!valid) {
        cfw_inflate_reset(stream);
        stream->context_valid = 0; /* only an explicit record reset can recover */
    }
    int result = cfw_message_reply(stream, here, origin,
        valid ? CFW_MESSAGE_ACK : CFW_MESSAGE_NACK, (uint16_t)size);
    if (decoded) CFW_MESSAGE_FREE(decoded);
    if (stream->buffer) CFW_MESSAGE_FREE(stream->buffer);
    stream->buffer = 0;
    stream->size = stream->used = stream->length_bytes = 0;
    ++stream->message_id;
    return result == 0 ? 0 : 6;
}

/* The clear five-byte header may span packets. Allocate only after it arrives.
 * No packet boundary has any meaning to the record parser. */
static uint32_t cfw_message_process(const uint8_t *packet, uint16_t length,
                                    uint8_t here, uint8_t origin) {
    cfw_message_stream *stream = CFW_STREAM_STATE(origin);
    if (!stream) return 6;
    uint8_t options = packet[8] & CFW_MESSAGE_BOTH;
    if (packet[8] & CFW_MESSAGE_RESET) {
        if (stream->length_bytes) cfw_message_abort(stream, here, origin);
        cfw_message_discard(stream);
        stream->active = 1;
        stream->next_sequence = stream->stream_id = packet[2];
        stream->message_id = 0;
        stream->options = options;
        stream->packet_capacity = CFW_ACK_SIZE;
    }
    if (!stream->active || stream->next_sequence != packet[2] || stream->options != options) {
        cfw_message_abort(stream, here, origin);
        cfw_message_discard(stream);
        return 0xau;
    }
    ++stream->next_sequence; /* uint8 wrap is intentional. */
    /* Replies must still fit one ATT notification, including on MTU 23 links.
     * An observed request proves the ingress can carry that many bytes. Track
     * this per compression context (reset on reconnect), capped at 40 bytes
     * including the reply envelope. The bridge sees the same request sizes. */
    /* Stock TPL reserves 11 bytes per fragment, including its CRC allowance. */
    unsigned capacity = length - 11;
    if (capacity > CFW_ACK_MAX_SIZE) capacity = CFW_ACK_MAX_SIZE;
    if (capacity > stream->packet_capacity) stream->packet_capacity = capacity;
    uint32_t status = 0;
    uint16_t end = length - 2, cursor = 9;
    while (cursor < end) {
        if (stream->length_bytes < 5) {
            uint8_t byte = packet[cursor++];
            switch (stream->length_bytes++) {
            case 0: stream->flags = byte; break;
            case 1: stream->size = byte; break;
            case 2: stream->size |= (uint16_t)byte << 8; break;
            case 3: stream->checksum = byte; break;
            case 4: stream->checksum |= (uint16_t)byte << 8; break;
            }
            if (stream->length_bytes < 5) continue;
            if (stream->size) {
                stream->buffer = CFW_MESSAGE_MALLOC(stream->size);
                if (!stream->buffer) {
                    cfw_message_abort(stream, here, origin);
                    return 6; /* Cannot resume at a guessed record boundary. */
                }
            }
        }
        uint16_t count = stream->size - stream->used;
        if (count > end - cursor) count = end - cursor;
        if (count) memcpy(stream->buffer + stream->used, packet + cursor, count);
        stream->used += count;
        cursor += count;
        if (stream->used == stream->size) {
            uint32_t result = cfw_message_complete(stream, here, origin);
            if (result) status = result;
        }
    }
    if (packet[8] & CFW_MESSAGE_END) {
        if (stream->length_bytes) {
            cfw_message_abort(stream, here, origin);
            status = 0xbu; /* Truncated header or payload. */
        }
        cfw_message_discard(stream);
    }
    return status;
}

uint32_t cfw_receive_packet(uint8_t pipe, const uint8_t *packet, uint16_t length) {
    if (pipe != 0 || !packet || length < 8 || packet[0] != 0xaa || packet[6] != CFW_MESSAGE_SID)
        return CFW_STOCK_RECEIVE(pipe, packet, length);
    /* Consume malformed private packets before the stock multipart allocator. */
    uint32_t status = cfw_message_validate(packet, length);
    if (status) {
        uint8_t here = cfw_message_lens();
        if (here) cfw_message_abort(CFW_STREAM_STATE(here), here, here);
        return status;
    }
    uint8_t here = cfw_message_lens();
    if (!here) return 6;
    if (packet[8] & (here ^ CFW_MESSAGE_BOTH))
        if (cfw_message_bridge_send(CFW_BRIDGE_REQUEST, here, packet, length) != 0) status = 6;
    /* A failed peer enqueue does not prevent the selected local lens processing. */
    if (packet[8] & here) {
        uint32_t local_status = cfw_message_process(packet, length, here, here);
        if (local_status) status = local_status;
    }
    return status;
}

/* Intercept bridge delivery BEFORE SendUserDataToThreadPool. Its eight workers
 * can reorder callbacks; reconstruction instead runs on ordered bridge arrival.
 * Ordinary app IDs retain the original worker dispatch. The buffer is borrowed
 * only until return. Local echoes never process or forward a request. */
uint32_t cfw_message_bridge_received(uint32_t app_id, const uint8_t *data,
                                     uint32_t length, uint16_t event) {
    if (app_id != CFW_MESSAGE_SID)
        return CFW_STOCK_BRIDGE_RECEIVE(app_id, data, length, event);
    if (!data || length < 2) return 0xbu;
    uint8_t here = cfw_message_lens(), origin = data[1];
    if (!here || (origin != CFW_MESSAGE_LEFT && origin != CFW_MESSAGE_RIGHT)) return 0xau;
    if (data[0] == CFW_BRIDGE_REQUEST) {
        if (here == origin) return 0;
        if (length > 265) return 0xbu;
        uint32_t status = cfw_message_validate(data + 2, (uint16_t)(length - 2));
        if (status) {
            cfw_message_abort(CFW_STREAM_STATE(origin), here, origin);
            return status;
        }
        return (data[10] & here) ? cfw_message_process(data + 2, length - 2, here, origin) : 0;
    }
    if (data[0] == CFW_BRIDGE_RETURN) {
        if (here != origin) return 0;
        if (length < CFW_ACK_SIZE + 2 || length > CFW_ACK_MAX_SIZE + 2 ||
            (length - CFW_ACK_SIZE - 2) % CFW_ACK_ENTRY_SIZE != 0) return 0xbu;
        if ((data[2] != CFW_MESSAGE_ACK && data[2] != CFW_MESSAGE_NACK) || data[6] != (origin ^ CFW_MESSAGE_BOTH)) return 0xau;
        if (data[2] == CFW_MESSAGE_NACK && length != CFW_ACK_SIZE + 2) return 0xbu;
        return CFW_BLE_SEND(1, CFW_MESSAGE_SID, data + 2, length - 2) == 0 ? 0 : 6;
    }
    return 0xau;
}
