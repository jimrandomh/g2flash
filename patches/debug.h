#pragma once
#include <stdint.h>

/* Per-frame list of updated rectangles (pixel coords), assembled on the stack of the
 * top-level image_worker and threaded through image_dispatch; present_shadow outlines
 * them in the display buffer when the debug overlay is on, to visualize update regions. */
#define CFW_RECT_MAX 16
typedef struct { uint16_t l, t, w, h; } cfw_rect;
typedef struct {
    uint32_t n;
    uint8_t direct_submitted;
    cfw_rect r[CFW_RECT_MAX];
} cfw_rectlist;

static void cfw_time_start(uint32_t *t);
static uint32_t cfw_time_end(const uint32_t *t);
static void rl_add(cfw_rectlist *rl, uint32_t l, uint32_t t, uint32_t w, uint32_t h);
static int cfw_diag(int has_fid, uint16_t fid);
static void cfw_draw_flags(uint8_t *disp, uint32_t w, uint32_t h);
