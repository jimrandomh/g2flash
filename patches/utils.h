#pragma once
#include <stdint.h>

static void bzero(uint8_t *buf, uint32_t len);
static uint32_t strlcpy(char *dst, const char *src, uint32_t len);
static uint32_t strnlen(const char *s, uint32_t maxlen);
static uint32_t strlcat(char *dst, const char *src, uint32_t len);

static uint32_t rd16(const uint8_t *p);
static uint32_t rd32(const uint8_t *p);
