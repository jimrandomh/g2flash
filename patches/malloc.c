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

/* Allocate from the independent 820 KiB TLSF arena at 0x2013be70. Go through
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


static uint32_t tlsf_arena_free(uint32_t arena, uint32_t arena_size) {
    uint32_t arena_end = arena + arena_size;
    if ((arena & 3u) || arena_end < arena || arena_size < TLSF_CONTROL_BYTES + 8u)
        return TLSF_FREE_INVALID;

    uint32_t size_word = arena + TLSF_CONTROL_BYTES;
    uint32_t free_bytes = 0;
    uint32_t max_blocks = (arena_size - TLSF_CONTROL_BYTES - 4u) / 16u + 1u;
    for (uint32_t n = 0; n < max_blocks; n++) {
        if ((size_word & 3u) || size_word > arena_end - 4u)
            return TLSF_FREE_INVALID;

        uint32_t size_flags = *(volatile uint32_t *)(uintptr_t)size_word;
        uint32_t block_size = size_flags & ~3u;
        if (block_size == 0)
            return size_word == arena_end - 4u ? free_bytes : TLSF_FREE_INVALID;
        if (size_word > arena_end - 8u || block_size < TLSF_BLOCK_MIN ||
            block_size > arena_end - size_word - 8u)
            return TLSF_FREE_INVALID;

        if (size_flags & 1u) free_bytes += block_size;
        size_word += block_size + 4u;
    }
    return TLSF_FREE_INVALID;
}

/* Generic heap descriptors made by FUN_0048413c contain their TLSF pointer at
 * +4, arena size at +0x10, and arena base at +0x14. Check all three before
 * trusting the pool. The policy byte at +0x18 belongs to the higher selector. */
static uint32_t heap_object_free(uint32_t descriptor, uint32_t arena,
                                 uint32_t arena_size) {
    volatile uint32_t *heap = (volatile uint32_t *)(uintptr_t)descriptor;
    if (heap[1] != arena || heap[4] != arena_size || heap[5] != arena)
        return TLSF_FREE_INVALID;
    return tlsf_arena_free(arena, arena_size);
}
