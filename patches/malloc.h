#pragma once

typedef void *(*malloc_fn)(uint32_t);
typedef void (*free_fn)(void *);

/* Stock mutex-protected EvenHub TLSF wrappers. Both load the arena pointer
 * from 0x20077ed0. Every CFW allocation comes from this heap (see ram_layout.h
 * for its bounds). Nothing of ours lives in the LVGL arena, which is not cached
 * and so is slow for buffers that only the CPU touches. */
#define FW_MALLOC  ((malloc_fn)0x0045855fU)         /* FUN_0045855e malloc(size) */
#define FW_FREE    ((free_fn)0x004585a3U)           /* FUN_004585a2 free(ptr) */

static void *cfw_malloc(uint32_t size);
typedef struct {
    uint32_t free_bytes;
    uint32_t max_alloc;
} cfw_heap_stats;
static cfw_heap_stats tlsf_arena_stats(uint32_t arena, uint32_t arena_size);
static cfw_heap_stats heap_object_stats(uint32_t descriptor, uint32_t arena, uint32_t arena_size);

/* The stock TLSF build is the 32-bit, 4-byte-aligned configuration. A pool made
 * by tlsf_create_with_pool() starts after its 0xc74-byte control structure. Its
 * physical block chain has a size/status word at the pool address, then another
 * size/status word every (block size + 4) bytes, and ends with a zero-size word
 * at arena_end - 4. Sum the payload capacity of free blocks and find the largest
 * ordinary (4-byte-aligned) malloc request, validating every
 * step so an uninitialized pool or a concurrent split/coalesce produces "?"
 * instead of an out-of-arena read or a bogus free-space value.
 *
 * TLSF is not itself thread-safe. These diagnostics deliberately do not take the
 * allocator mutexes: they run in the display path, must not stall it, and a
 * validated approximate snapshot is preferable to introducing lock ordering
 * into that path. The aligned 32-bit metadata reads are atomic on this core. */
#define TLSF_CONTROL_BYTES 0x0c74U
#define TLSF_BLOCK_MIN     12U
#define TLSF_FREE_INVALID  0xffffffffU
