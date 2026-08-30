#pragma once

static unsigned pb_varint_size(unsigned value);
static unsigned pb_write_varint(unsigned char *p, unsigned value);
static unsigned pb_append_bytes_field(unsigned char *buf, unsigned len, unsigned capacity, unsigned field_number, const unsigned char *data, unsigned data_len);

