#pragma once
#include <stddef.h>

/* Normal memory only (not volatile/MMIO). memcpy requires disjoint ranges;
 * memmove also supports overlap. Both return the original destination. */
void *(memcpy)(void *restrict dst, const void *restrict src, size_t size);
void *(memmove)(void *dst, const void *src, size_t size);
