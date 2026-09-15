#include <stdint.h>
#include "malloc.h"
#include "cfw_context.h"

/* Keep the allocation diagnostic outside customCfwContext so failure to allocate
 * that context is itself observable. The magic rejects uninitialized/warm-reset
 * SRAM; bit 0 is sticky until mode 7/subcommand 0 clears the diagnostics. */
static uint32_t cfw_alloc_diag(void) {
    volatile uint32_t *slot = (volatile uint32_t *)CFW_ALLOC_DIAG_SLOT;
    uint32_t v = *slot;
    if ((v & ~1u) != CFW_ALLOC_DIAG_MAGIC) {
        v = CFW_ALLOC_DIAG_MAGIC;
        *slot = v;
    }
    return v;
}

static void cfw_alloc_diag_clear(void) {
    *(volatile uint32_t *)CFW_ALLOC_DIAG_SLOT = CFW_ALLOC_DIAG_MAGIC;
}

__attribute__((noinline)) static void *cfw_malloc(uint32_t size) {
    cfw_alloc_diag();
    void *p = FW_MALLOC(size);
    if (p == 0)
        *(volatile uint32_t *)CFW_ALLOC_DIAG_SLOT = CFW_ALLOC_DIAG_MAGIC | 1u;
    return p;
}

/* Allocate from the independent 820 KiB TLSF arena at 0x201350a8. Go through
 * the stock generic heap coordinator rather than calling TLSF directly so the
 * descriptor's mutex, current-byte counter, and peak-byte counter stay valid. */
__attribute__((noinline)) static void *cfw_heap13_malloc(uint32_t size) {
    cfw_alloc_diag();
    void *p = FW_HEAP_MALLOC(FW_HEAP_13_DESCRIPTOR, size);
    if (p == 0)
        *(volatile uint32_t *)CFW_ALLOC_DIAG_SLOT = CFW_ALLOC_DIAG_MAGIC | 1u;
    return p;
}

__attribute__((noinline)) static void cfw_heap13_free(void *ptr) {
    FW_HEAP_FREE(FW_HEAP_13_DESCRIPTOR, ptr);
}


#ifndef CFW_HEAP_READ32
#define CFW_HEAP_READ32(addr) (*(volatile uint32_t *)(uintptr_t)(addr))
#endif

/* Stock mapping_search (0x49ae8e) rounds requests up to a TLSF bin;
 * mapping_insert (0x49ae60) rounds free blocks down. There are 32 bins per
 * power of two, with 4-byte bins below 128 bytes. A request above the lower
 * edge of the largest occupied bin skips that bin, even if its block fits.
 * Generic heap malloc (0x48c1e8) adds no per-request header of its own. */
static uint32_t tlsf_max_request(uint32_t block_size) {
    uint32_t step = 4u;
    for (uint32_t n = block_size; n >= 256u; n >>= 1) step <<= 1;
    return block_size & ~(step - 1u);
}

static cfw_heap_stats tlsf_arena_stats(uint32_t arena, uint32_t arena_size) {
    const cfw_heap_stats invalid = {TLSF_FREE_INVALID, TLSF_FREE_INVALID};
    uint32_t arena_end = arena + arena_size;
    if ((arena & 3u) || arena_end < arena || arena_size < TLSF_CONTROL_BYTES + 8u)
        return invalid;

    uint32_t size_word = arena + TLSF_CONTROL_BYTES;
    cfw_heap_stats stats = {0, 0};
    uint32_t largest_block = 0;
    uint32_t max_blocks = (arena_size - TLSF_CONTROL_BYTES - 4u) / 16u + 1u;
    for (uint32_t n = 0; n < max_blocks; n++) {
        if ((size_word & 3u) || size_word > arena_end - 4u)
            return invalid;

        uint32_t size_flags = CFW_HEAP_READ32(size_word);
        uint32_t block_size = size_flags & ~3u;
        if (block_size == 0) {
            stats.max_alloc = tlsf_max_request(largest_block);
            return size_word == arena_end - 4u ? stats : invalid;
        }
        if (size_word > arena_end - 8u || block_size < TLSF_BLOCK_MIN ||
            block_size > arena_end - size_word - 8u)
            return invalid;

        if (size_flags & 1u) {
            stats.free_bytes += block_size;
            if (block_size > largest_block) largest_block = block_size;
        }
        size_word += block_size + 4u;
    }
    return invalid;
}

/* Generic heap descriptors made by the stock heap constructor contain their
 * TLSF pointer at +4, arena size at +0x10, and arena base at +0x14. Check all
 * three before trusting the pool. The policy byte at +0x18 belongs to the
 * higher selector. */
static cfw_heap_stats heap_object_stats(uint32_t descriptor, uint32_t arena,
                                        uint32_t arena_size) {
    if (CFW_HEAP_READ32(descriptor + 4u) != arena ||
        CFW_HEAP_READ32(descriptor + 16u) != arena_size ||
        CFW_HEAP_READ32(descriptor + 20u) != arena) {
        const cfw_heap_stats invalid = {TLSF_FREE_INVALID, TLSF_FREE_INVALID};
        return invalid;
    }
    return tlsf_arena_stats(arena, arena_size);
}
