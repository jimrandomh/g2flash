#include <stdint.h>
#include "memory.h"
#include "cfw_context.h"
#include "resource_cache.h"

static uint32_t cfw_resource_span(uint32_t length) {
    return sizeof(cfw_resource_block) + ((length + 3u) & ~3u);
}
static cfw_resource_block *cfw_resource_block_at(customCfwContext *ctx, uint32_t offset) {
    return (cfw_resource_block *)(ctx->resource_cache + offset);
}
static void cfw_resource_publish(customCfwContext *ctx, cfw_resource_block *block) {
    /* Actual 32-bit pointers on the target. Subtracting the low base word in
     * get() also lets the exact allocator run under 64-bit host sanitizers. */
    ((uint32_t *)ctx->resource_cache)[block->id] = block->received == block->length
        ? (uint32_t)(uintptr_t)(block + 1) : 0;
}
static cfw_resource_block *cfw_resource_find(customCfwContext *ctx, uint32_t id) {
    if (!ctx || !ctx->resource_cache || id >= CFW_RESOURCE_COUNT) return 0;
    for (uint32_t pos = CFW_RESOURCE_TABLE_BYTES; pos < CFW_RESOURCE_CACHE_SIZE;) {
        cfw_resource_block *b = cfw_resource_block_at(ctx, pos);
        if (b->id == id) return b;
        pos += b->span;
    }
    return 0;
}
static const uint8_t *cfw_resource_get(customCfwContext *ctx, uint32_t id, uint32_t *size) {
    if (!ctx || !ctx->resource_cache || id >= CFW_RESOURCE_COUNT) return 0;
    uint32_t pointer = ((uint32_t *)ctx->resource_cache)[id];
    if (!pointer) return 0;
    uint32_t offset = pointer - (uint32_t)(uintptr_t)ctx->resource_cache;
    if ((offset & 3u) || offset < CFW_RESOURCE_TABLE_BYTES + sizeof(cfw_resource_block) ||
        offset > CFW_RESOURCE_CACHE_SIZE) return 0;
    cfw_resource_block *b = cfw_resource_block_at(ctx, offset - sizeof(cfw_resource_block));
    if (b->id != id || !b->length || b->length > CFW_RESOURCE_MAX_SIZE ||
        b->received != b->length || b->length > CFW_RESOURCE_CACHE_SIZE - offset) return 0;
    *size = b->length;
    return ctx->resource_cache + offset;
}
static uint32_t cfw_resource_used(customCfwContext *ctx) {
    uint32_t used = 0;
    if (!ctx || !ctx->resource_cache) return 0;
    for (uint32_t pos = CFW_RESOURCE_TABLE_BYTES; pos < CFW_RESOURCE_CACHE_SIZE;) {
        cfw_resource_block *b = cfw_resource_block_at(ctx, pos);
        if (b->id != CFW_RESOURCE_FREE) used += cfw_resource_span(b->length);
        pos += b->span;
    }
    return used;
}
static void cfw_resource_rebuild_free(customCfwContext *ctx) {
    ctx->resource_free_head = 0;
    cfw_resource_block *previous = 0;
    for (uint32_t pos = CFW_RESOURCE_TABLE_BYTES; pos < CFW_RESOURCE_CACHE_SIZE;) {
        cfw_resource_block *b = cfw_resource_block_at(ctx, pos);
        if (b->id == CFW_RESOURCE_FREE) {
            uint32_t next = pos + b->span;
            while (next < CFW_RESOURCE_CACHE_SIZE) {
                cfw_resource_block *n = cfw_resource_block_at(ctx, next);
                if (n->id != CFW_RESOURCE_FREE) break;
                b->span += n->span;
                next += n->span;
            }
            b->length = 0;
            if (previous) previous->length = pos;
            else ctx->resource_free_head = pos;
            previous = b;
        }
        pos += b->span;
    }
}
static void cfw_resource_compact(customCfwContext *ctx) {
    uint32_t write = CFW_RESOURCE_TABLE_BYTES;
    cfw_resource_block *last = 0;
    for (uint32_t read = CFW_RESOURCE_TABLE_BYTES; read < CFW_RESOURCE_CACHE_SIZE;) {
        cfw_resource_block *b = cfw_resource_block_at(ctx, read);
        uint32_t old_span = b->span;
        if (b->id != CFW_RESOURCE_FREE) {
            uint32_t span = cfw_resource_span(b->length);
            cfw_resource_block *moved = cfw_resource_block_at(ctx, write);
            memmove(moved, b, sizeof(*b) + b->received);
            moved->span = span;
            cfw_resource_publish(ctx, moved);
            last = moved;
            write += span;
        }
        read += old_span;
    }
    ctx->resource_free_head = 0;
    uint32_t remaining = CFW_RESOURCE_CACHE_SIZE - write;
    if (remaining >= sizeof(cfw_resource_block)) {
        cfw_resource_block *free = cfw_resource_block_at(ctx, write);
        free->span = remaining; free->length = 0;
        free->id = CFW_RESOURCE_FREE; free->received = 0;
        ctx->resource_free_head = write;
    } else if (last) last->span += remaining;
}
static cfw_resource_block *cfw_resource_allocate(customCfwContext *ctx, uint32_t id, uint32_t length) {
    uint32_t need = cfw_resource_span(length);
    for (unsigned attempt = 0; attempt < 2; attempt++) {
        uint32_t pos = ctx->resource_free_head;
        cfw_resource_block *previous = 0;
        while (pos) {
            cfw_resource_block *b = cfw_resource_block_at(ctx, pos);
            uint32_t next = b->length;
            if (b->span >= need) {
                if (b->span - need >= sizeof(*b)) {
                    cfw_resource_block *tail = cfw_resource_block_at(ctx, pos + need);
                    tail->span = b->span - need; tail->length = next;
                    tail->id = CFW_RESOURCE_FREE; tail->received = 0;
                    next = pos + need;
                    b->span = need;
                }
                if (previous) previous->length = next;
                else ctx->resource_free_head = next;
                b->length = length; b->id = id; b->received = 0;
                return b;
            }
            previous = b; pos = next;
        }
        cfw_resource_compact(ctx);
    }
    return 0; /* Preflight has already checked capacity. */
}
static void cfw_resource_cache_release(customCfwContext *ctx) {
    if (ctx) {
        uint8_t *cache = ctx->resource_cache;
        ctx->resource_cache = 0;
        ctx->resource_free_head = 0;
        if (cache) FW_FREE(cache);
    }
}
static int cfw_resource_upload(const uint8_t *src, uint32_t len) {
    if (!src || len < 2) return -1;
    uint32_t count = rd16(src), pos = 2;
    if (count > CFW_RESOURCE_COUNT) return -1;
    customCfwContext *ctx = getCustomCfwContext();
    /* Lease validation can release expired storage, so it precedes every lookup. */
    if (count && (!cfw_fb_lease_active() || !ctx)) return -1;
    volatile uint8_t seen[CFW_RESOURCE_COUNT / 8];
    for (uint32_t i = 0; i < sizeof(seen); i++) seen[i] = 0;
    uint32_t used = cfw_resource_used(ctx);
    for (uint32_t i = 0; i < count; i++) {
        if (len - pos < 10) return -1;
        uint32_t id = rd16(src + pos), total = rd32(src + pos + 2);
        uint32_t offset = rd16(src + pos + 6), size = rd16(src + pos + 8);
        pos += 10;
        if (id >= CFW_RESOURCE_COUNT || !total || total > CFW_RESOURCE_MAX_SIZE ||
            !size || size > len - pos || offset > total || size > total - offset ||
            (seen[id >> 3] & (1u << (id & 7u)))) return -1;
        seen[id >> 3] |= 1u << (id & 7u);
        cfw_resource_block *b = cfw_resource_find(ctx, id);
        if (!b) {
            if (offset) return -1;
            used += cfw_resource_span(total);
        } else {
            if (b->length != total || offset > b->received) return -1;
            if (offset < b->received) {
                if (size > b->received - offset) return -1;
                const uint8_t *data = (const uint8_t *)(b + 1) + offset;
                for (uint32_t j = 0; j < size; j++) if (data[j] != src[pos + j]) return -1;
            }
        }
        pos += size;
    }
    if (pos != len || used > CFW_RESOURCE_CACHE_SIZE - CFW_RESOURCE_TABLE_BYTES) return -1;
    if (!count) return 0;
    if (!ctx->resource_cache) {
        uint8_t *cache = (uint8_t *)cfw_malloc(CFW_RESOURCE_CACHE_SIZE);
        if (!cache) return -1;
        bzero(cache, CFW_RESOURCE_TABLE_BYTES);
        ctx->resource_cache = cache;
        cfw_resource_block *free = cfw_resource_block_at(ctx, CFW_RESOURCE_TABLE_BYTES);
        free->span = CFW_RESOURCE_CACHE_SIZE - CFW_RESOURCE_TABLE_BYTES;
        free->length = 0; free->id = CFW_RESOURCE_FREE; free->received = 0;
        ctx->resource_free_head = CFW_RESOURCE_TABLE_BYTES;
    }
    pos = 2;
    for (uint32_t i = 0; i < count; i++) {
        uint32_t id = rd16(src + pos), total = rd32(src + pos + 2);
        uint32_t offset = rd16(src + pos + 6), size = rd16(src + pos + 8);
        pos += 10;
        cfw_resource_block *b = cfw_resource_find(ctx, id);
        if (!b) b = cfw_resource_allocate(ctx, id, total);
        if (!b) return -1;
        if (offset == b->received) {
            memcpy((uint8_t *)(b + 1) + offset, src + pos, size);
            b->received += size;
            cfw_resource_publish(ctx, b);
        }
        pos += size;
    }
    return 0;
}
static int cfw_resource_evict(const uint8_t *src, uint32_t len) {
    if (!src || len < 2) return -1;
    uint32_t count = rd16(src);
    if (count > CFW_RESOURCE_COUNT || len != 2 + count * 2) return -1;
    volatile uint8_t seen[CFW_RESOURCE_COUNT / 8];
    for (uint32_t i = 0; i < sizeof(seen); i++) seen[i] = 0;
    for (uint32_t i = 0; i < count; i++) {
        uint32_t id = rd16(src + 2 + i * 2);
        if (id >= CFW_RESOURCE_COUNT || (seen[id >> 3] & (1u << (id & 7u)))) return -1;
        seen[id >> 3] |= 1u << (id & 7u);
    }
    if (!count) return 0;
    if (!cfw_fb_lease_active()) return -1;
    customCfwContext *ctx = getCustomCfwContext();
    if (!ctx) return -1;
    if (!ctx->resource_cache) return 0;
    for (uint32_t pos = CFW_RESOURCE_TABLE_BYTES; pos < CFW_RESOURCE_CACHE_SIZE;) {
        cfw_resource_block *b = cfw_resource_block_at(ctx, pos);
        if (b->id < CFW_RESOURCE_COUNT && (seen[b->id >> 3] & (1u << (b->id & 7u)))) {
            ((uint32_t *)ctx->resource_cache)[b->id] = 0;
            b->id = CFW_RESOURCE_FREE; b->received = 0;
        }
        pos += b->span;
    }
    cfw_resource_rebuild_free(ctx);
    return 0;
}
