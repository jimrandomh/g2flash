#include "protobuf.h"

/* Minimal protobuf helpers for appending length-delimited extension fields to
 * an existing message. All capacity checks happen before the buffer is touched.
 */
static unsigned pb_varint_size(unsigned value) {
    unsigned size = 1;
    while (value >= 0x80u) {
        value >>= 7;
        size++;
    }
    return size;
}

static unsigned pb_write_varint(unsigned char *p, unsigned value) {
    unsigned size = 0;
    do {
        unsigned char byte = (unsigned char)(value & 0x7fu);
        value >>= 7;
        p[size++] = (unsigned char)(byte | (value ? 0x80u : 0u));
    } while (value);
    return size;
}

/* Return the new message length, or the original length if the field does not
 * fit. Returning the original length lets callers safely try another optional
 * field without ever constructing a partial protobuf record.
 */
static unsigned pb_append_bytes_field(unsigned char *buf, unsigned len,
                                      unsigned capacity, unsigned field_number,
                                      const unsigned char *data, unsigned data_len) {
    unsigned tag = (field_number << 3) | 2u;
    unsigned tag_size = pb_varint_size(tag);
    unsigned len_size = pb_varint_size(data_len);

    if (len > capacity) return len;
    unsigned remaining = capacity - len;
    if (tag_size > remaining) return len;
    remaining -= tag_size;
    if (len_size > remaining) return len;
    remaining -= len_size;
    if (data_len > remaining) return len;

    unsigned char *p = buf + len;
    p += pb_write_varint(p, tag);
    p += pb_write_varint(p, data_len);
    for (unsigned i = 0; i < data_len; i++) p[i] = data[i];
    return len + tag_size + len_size + data_len;
}

