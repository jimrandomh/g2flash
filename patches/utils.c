#include "utils.h"

static void bzero(uint8_t *buf, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) buf[i] = 0;
}
static uint32_t strlcpy(char *dst, const char *src, uint32_t len) {
  if (len == 0) {
      return 0;
  }
  for (uint32_t i=0; i<len; i++) {
    dst[i] = src[i];
    if (src[i] == 0) {
      return i;
    }
  }
  dst[len-1] = 0;
  return len;
}

static uint32_t strnlen(const char *s, uint32_t maxlen) {
  for (uint32_t i=0; i<maxlen; i++) {
    if (s[i] == 0) {
      return i;
    }
  }
  return maxlen;
}

static uint32_t strlcat(char *dst, const char *src, uint32_t len) {
    uint32_t i = strnlen(dst, len), j = 0;
    for (; i < len; i++, j++) {
        dst[i] = src[j];
        if (src[j] == 0) return i;
    }
    if (len) dst[len-1] = 0;
    return len;
}


/* little-endian unaligned reads (byte-wise; -mno-unaligned-access safe) */
static uint32_t rd16(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8);
}

static uint32_t rd32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}