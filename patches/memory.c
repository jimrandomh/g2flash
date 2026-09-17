#include "memory.h"
#include <stdint.h>

/* Scalar Thumb-2 for the Apollo510's Cortex-M55, without depending on FP/MVE
 * context or unaligned-access settings in the stock firmware. Equal alignment
 * lets us peel bytes then copy aligned words; differing alignment uses bytes.
 * The 16-byte memcpy blocks let Clang select aligned ARM word loads/stores.
 *
 * Unlike __builtin_memcpy with a runtime size, __builtin_memcpy_inline cannot
 * call memcpy (or an ABI helper). no_builtin also prevents loop recognition
 * from introducing recursive library calls. Keep the exported implementations
 * out of line so every caller shares the bulk-copy machinery.
 */
__attribute__((noinline, no_builtin("memcpy", "memmove")))
void *memcpy(void *restrict dst, const void *restrict src, size_t size) {
    uint8_t *d = dst;
    const uint8_t *s = src;
    if ((((uintptr_t)d ^ (uintptr_t)s) & 3u) == 0) {
        #pragma clang loop unroll(disable)
        while (size && ((uintptr_t)d & 3u)) {
            *d++ = *s++;
            --size;
        }
        #pragma clang loop unroll(disable)
        while (size >= 16) {
            __builtin_memcpy_inline(__builtin_assume_aligned(d, 4),
                                    __builtin_assume_aligned(s, 4), 16);
            d += 16;
            s += 16;
            size -= 16;
        }
        #pragma clang loop unroll(disable)
        while (size >= 4) {
            __builtin_memcpy_inline(__builtin_assume_aligned(d, 4),
                                    __builtin_assume_aligned(s, 4), 4);
            d += 4;
            s += 4;
            size -= 4;
        }
    }
    #pragma clang loop unroll(disable)
    while (size) {
        *d++ = *s++;
        --size;
    }
    return dst;
}

__attribute__((noinline, no_builtin("memcpy", "memmove")))
void *memmove(void *dst, const void *src, size_t size) {
    uint8_t *d = dst;
    const uint8_t *s = src;
    uintptr_t da = (uintptr_t)d, sa = (uintptr_t)s;
    if (!size || d == s) return dst;
    /* Integer comparisons support separate objects on our flat address space.
     * Subtract the smaller address to avoid end-address overflow. */
    if ((da < sa ? sa - da : da - sa) >= size)
        return memcpy(dst, src, size);

    if (da < sa) {
        if (((da ^ sa) & 3u) == 0) {
            #pragma clang loop unroll(disable)
            while (size && ((uintptr_t)d & 3u)) {
                *d++ = *s++;
                --size;
            }
            /* Equal alignment and distinct pointers imply a gap >= 4, so
             * each individual word copy is disjoint even when ranges overlap.
             * Builtins avoid type-punning/strict-aliasing violations. */
            #pragma clang loop unroll(disable)
            while (size >= 4) {
                __builtin_memcpy_inline(__builtin_assume_aligned(d, 4),
                                        __builtin_assume_aligned(s, 4), 4);
                d += 4;
                s += 4;
                size -= 4;
            }
        }
        #pragma clang loop unroll(disable)
        while (size) {
            *d++ = *s++;
            --size;
        }
    } else {
        d += size;
        s += size;
        if (((da ^ sa) & 3u) == 0) {
            #pragma clang loop unroll(disable)
            while (size && ((uintptr_t)d & 3u)) {
                *--d = *--s;
                --size;
            }
            #pragma clang loop unroll(disable)
            while (size >= 4) {
                d -= 4;
                s -= 4;
                __builtin_memcpy_inline(__builtin_assume_aligned(d, 4),
                                        __builtin_assume_aligned(s, 4), 4);
                size -= 4;
            }
        }
        #pragma clang loop unroll(disable)
        while (size) {
            *--d = *--s;
            --size;
        }
    }
    return dst;
}
