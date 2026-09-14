#include "message_transport.h"

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

#ifndef CFW_STREAM_STATE
static cfw_message_stream *cfw_message_state(uint8_t origin) {
    customCfwContext *ctx = getCustomCfwContext();
    return ctx ? &ctx->message_streams[origin - 1] : 0;
}
#define CFW_STREAM_STATE cfw_message_state
#define CFW_MESSAGE_MALLOC cfw_heap13_malloc
#define CFW_MESSAGE_FREE cfw_heap13_free
#endif

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
    for (uint16_t i = 0; i < length; ++i) envelope[i + 2] = body[i];
    return CFW_BRIDGE_SEND(CFW_MESSAGE_SID, envelope, length + 2, 0);
}

static void cfw_message_discard(cfw_message_stream *stream) {
    if (stream->buffer) CFW_MESSAGE_FREE(stream->buffer);
    stream->buffer = 0;
    stream->size = stream->used = 0;
    stream->length_bytes = stream->active = 0;
}

/* ACK identifies the stream and message ordinal, independent of packet splits.
 * Several messages ending in one packet must still have distinct ACKs. */
static uint32_t cfw_message_complete(cfw_message_stream *stream, uint8_t here,
                                     uint8_t origin) {
    uint8_t empty = 0;
    const uint8_t *data = stream->buffer ? stream->buffer : &empty;
    uint16_t crc = cfw_message_crc(data, stream->size);
    int result = cfw_message_received(data, stream->size, crc);
    if (result == 0) {
        uint8_t ack[CFW_ACK_SIZE] = {CFW_MESSAGE_ACK, stream->stream_id,
            (uint8_t)stream->message_id, (uint8_t)(stream->message_id >> 8), here,
            (uint8_t)stream->size, (uint8_t)(stream->size >> 8),
            (uint8_t)crc, (uint8_t)(crc >> 8)};
        result = here == origin ? CFW_BLE_SEND(1, CFW_MESSAGE_SID, ack, sizeof(ack)) :
            cfw_message_bridge_send(CFW_BRIDGE_RETURN, origin, ack, sizeof(ack));
    }
    if (stream->buffer) CFW_MESSAGE_FREE(stream->buffer);
    stream->buffer = 0;
    stream->size = stream->used = stream->length_bytes = 0;
    ++stream->message_id;
    return result == 0 ? 0 : 6;
}

/* Every nonempty length allocates exactly that many bytes as soon as the second
 * tag byte arrives. No packet boundary has any meaning to the record parser. */
static uint32_t cfw_message_process(const uint8_t *packet, uint16_t length,
                                    uint8_t here, uint8_t origin) {
    cfw_message_stream *stream = CFW_STREAM_STATE(origin);
    if (!stream) return 6;
    uint8_t options = packet[8] & CFW_MESSAGE_BOTH;
    if (packet[8] & CFW_MESSAGE_RESET) {
        cfw_message_discard(stream);
        stream->active = 1;
        stream->next_sequence = stream->stream_id = packet[2];
        stream->message_id = 0;
        stream->options = options;
    }
    if (!stream->active || stream->next_sequence != packet[2] || stream->options != options) {
        cfw_message_discard(stream);
        return 0xau;
    }
    ++stream->next_sequence; /* uint8 wrap is intentional. */
    uint32_t status = 0;
    uint16_t end = length - 2, cursor = 9;
    while (cursor < end) {
        if (stream->length_bytes < 2) {
            uint8_t byte = packet[cursor++];
            if (stream->length_bytes == 0) stream->size = byte;
            else stream->size |= (uint16_t)byte << 8;
            if (++stream->length_bytes < 2) continue;
            if (stream->size) {
                stream->buffer = CFW_MESSAGE_MALLOC(stream->size);
                if (!stream->buffer) {
                    cfw_message_discard(stream);
                    return 6; /* Cannot resume at a guessed record boundary. */
                }
            }
        }
        uint16_t count = stream->size - stream->used;
        if (count > end - cursor) count = end - cursor;
        for (uint16_t i = 0; i < count; ++i)
            stream->buffer[stream->used + i] = packet[cursor + i];
        stream->used += count;
        cursor += count;
        if (stream->used == stream->size) {
            uint32_t result = cfw_message_complete(stream, here, origin);
            if (result) status = result;
        }
    }
    if (packet[8] & CFW_MESSAGE_END) {
        if (stream->length_bytes) status = 0xbu; /* Truncated length or payload. */
        cfw_message_discard(stream);
    }
    return status;
}

uint32_t cfw_receive_packet(uint8_t pipe, const uint8_t *packet, uint16_t length) {
    if (pipe != 0 || !packet || length < 8 || packet[0] != 0xaa || packet[6] != CFW_MESSAGE_SID)
        return CFW_STOCK_RECEIVE(pipe, packet, length);
    /* Consume malformed private packets before the stock multipart allocator. */
    uint32_t status = cfw_message_validate(packet, length);
    if (status) return status;
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
        if (status) return status;
        return (data[10] & here) ? cfw_message_process(data + 2, length - 2, here, origin) : 0;
    }
    if (data[0] == CFW_BRIDGE_RETURN) {
        if (here != origin) return 0;
        if (length != CFW_ACK_SIZE + 2) return 0xbu;
        if (data[2] != CFW_MESSAGE_ACK || data[6] != (origin ^ CFW_MESSAGE_BOTH)) return 0xau;
        return CFW_BLE_SEND(1, CFW_MESSAGE_SID, data + 2, CFW_ACK_SIZE) == 0 ? 0 : 6;
    }
    return 0xau;
}
