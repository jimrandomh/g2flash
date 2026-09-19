#pragma once

typedef void *(*malloc_fn)(uint32_t);
typedef void (*free_fn)(void *);
typedef void *(*heap_malloc_fn)(uint32_t descriptor, uint32_t size);
typedef void (*heap_free_fn)(uint32_t descriptor, void *ptr);

/* Stock mutex-protected EvenHub TLSF wrappers. Both load the arena pointer
 * from 0x20077ed0 (arena 0x2020219c, size 0x70800 in firmware 2.3.0.24). */
#define FW_MALLOC  ((malloc_fn)0x0045855fU)         /* FUN_0045855e malloc(size) */
#define FW_FREE    ((free_fn)0x004585a3U)           /* FUN_004585a2 free(ptr) */
#define FW_HEAP_MALLOC ((heap_malloc_fn)0x0048d981U) /* FUN_0048d980 generic heap malloc */
#define FW_HEAP_FREE   ((heap_free_fn)0x0048da9fU)   /* FUN_0048da9e generic heap free */
#define FW_HEAP_13_DESCRIPTOR 0x20000358U            /* TLSF arena @ 0x2013519c, 0xcd000 B */

static void *cfw_malloc(uint32_t size);
static void *cfw_heap13_malloc(uint32_t size);
static void cfw_heap13_free(void *ptr);
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
