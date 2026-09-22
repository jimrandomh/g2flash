#pragma once
#include <stdint.h>

/* Image/text draw options; mirrored in Faceclaw DrawFlags.kt. */
#define CFW_TEXTURE_OPT_BRIGHTNESS_MASK 15u
#define CFW_TEXTURE_OPT_TRANSPARENT 16u
#define CFW_TEXTURE_OPT_INVERSE 32u

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
