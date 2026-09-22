#pragma once
#include <stdint.h>

#define CFW_RESOURCE_CACHE_SIZE (192u * 1024u)
#define CFW_RESOURCE_COUNT 512u
#define CFW_RESOURCE_TABLE_BYTES (CFW_RESOURCE_COUNT * 4u)
#define CFW_RESOURCE_MAX_SIZE 65536u
#define CFW_RESOURCE_FREE 0xffffffffu

/* Resource header byte; mirrored in Faceclaw ResourceFlags.kt. */
enum {
    CFW_RESOURCE_TYPE_IMAGE = 0,
    CFW_RESOURCE_TYPE_FONT = 1,
    CFW_RESOURCE_TYPE_DISPLAY_LIST = 2,
    CFW_RESOURCE_TYPE_MASK = 3,
    CFW_RESOURCE_FLAG_LARGE = 4,
    CFW_RESOURCE_FLAG_RLE = 8,
    CFW_RESOURCE_IMAGE_FLAGS_MASK = CFW_RESOURCE_FLAG_LARGE | CFW_RESOURCE_FLAG_RLE,
};

/* Live: length = payload bytes. Free: length = next free block offset.
 * received is the contiguous upload prefix; incomplete resources are invisible. */
typedef struct {
    uint32_t span, length, id, received;
} cfw_resource_block;

static void cfw_resource_cache_release(customCfwContext *ctx);
static int cfw_resource_upload(const uint8_t *src, uint32_t len);
static int cfw_resource_evict(const uint8_t *src, uint32_t len);
static const uint8_t *cfw_resource_get(customCfwContext *ctx, uint32_t id, uint32_t *size);
