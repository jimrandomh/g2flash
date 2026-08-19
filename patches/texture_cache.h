#pragma once
#include <stdint.h>

#define CFW_TEXTURE_CACHE_SIZE 65536u

static void cfw_texture_cache_release(customCfwContext *ctx);
static int cfw_texture_cache_update(const uint8_t *src, uint32_t len);
static int cfw_texture_draw_image(uint8_t *shadow, uint32_t stride,
                                  uint32_t panel_w, uint32_t panel_h,
                                  const uint8_t *src, uint32_t len,
                                  cfw_rectlist *rl);
static int cfw_texture_draw_string(uint8_t *shadow, uint32_t stride,
                                   uint32_t panel_w, uint32_t panel_h,
                                   const uint8_t *src, uint32_t len,
                                   cfw_rectlist *rl);
static int cfw_builtin_draw_string(uint8_t *shadow, uint32_t stride,
                                   uint32_t panel_w, uint32_t panel_h,
                                   const uint8_t *src, uint32_t len,
                                   cfw_rectlist *rl);
