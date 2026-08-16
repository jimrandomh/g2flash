#pragma once

typedef void *(*malloc_fn)(uint32_t);
typedef void (*free_fn)(void *);
typedef void *(*heap_malloc_fn)(uint32_t descriptor, uint32_t size);
typedef void (*heap_free_fn)(uint32_t descriptor, void *ptr);

#define FW_MALLOC  ((malloc_fn)0x00474cd3U)         /* FUN_00474cd2 malloc(size) */
#define FW_FREE    ((free_fn)0x00474d17U)           /* FUN_00474d16 free(ptr) */
#define FW_HEAP_MALLOC ((heap_malloc_fn)0x00484181U) /* FUN_00484180 generic heap malloc */
#define FW_HEAP_FREE   ((heap_free_fn)0x0048429fU)   /* FUN_0048429e generic heap free */
#define FW_HEAP_13_DESCRIPTOR 0x20000354U            /* TLSF arena @ 0x2013be70, 0xcd000 B */

static void *cfw_malloc(uint32_t size);
static void *cfw_heap13_malloc(uint32_t size);
static void cfw_heap13_free(void *ptr);
static uint32_t tlsf_arena_free(uint32_t arena, uint32_t arena_size);
static uint32_t heap_object_free(uint32_t descriptor, uint32_t arena, uint32_t arena_size);

/* The stock TLSF build is the 32-bit, 4-byte-aligned configuration. A pool made
 * by tlsf_create_with_pool() starts after its 0xc74-byte control structure. Its
 * physical block chain has a size/status word at the pool address, then another
 * size/status word every (block size + 4) bytes, and ends with a zero-size word
 * at arena_end - 4. Sum the payload capacity of free blocks, validating every
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
