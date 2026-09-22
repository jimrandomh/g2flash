#pragma once
#include <stdint.h>

#define CFW_RESOURCE_CACHE_SIZE (192u * 1024u)
#define CFW_RESOURCE_COUNT 512u
#define CFW_RESOURCE_TABLE_BYTES (CFW_RESOURCE_COUNT * 4u)
#define CFW_RESOURCE_MAX_SIZE 65536u
#define CFW_RESOURCE_FREE 0xffffffffu

/* Live: length = payload bytes. Free: length = next free block offset.
 * received is the contiguous upload prefix; incomplete resources are invisible. */
typedef struct {
    uint32_t span, length, id, received;
} cfw_resource_block;

static void cfw_resource_cache_release(customCfwContext *ctx);
static int cfw_resource_upload(const uint8_t *src, uint32_t len);
static int cfw_resource_evict(const uint8_t *src, uint32_t len);
static const uint8_t *cfw_resource_get(customCfwContext *ctx, uint32_t id, uint32_t *size);
