#pragma once

static void set_pixel4(uint8_t *disp, uint32_t w, uint32_t x, uint32_t y, uint8_t v);
static void draw_glyph(uint8_t *disp, uint32_t w, uint32_t h, int x0, int y0, char ch, uint8_t fg);
static void draw_string(uint8_t *disp, uint32_t w, uint32_t h, int x0, int y0, const char *s, uint8_t fg, int bg);
static void rect_copy_4bpp(uint8_t *buf, uint32_t stride, uint32_t sL, uint32_t sT, uint32_t dL, uint32_t dT, uint32_t bw, uint32_t bh);
