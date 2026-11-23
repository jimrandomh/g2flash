#pragma once
#include <stdint.h>

/* The command task owns timer creation and the inbound PRESENT epoch, and
 * publishes one immutable request word. Fade state belongs to the display task. No panel register or persistent setting is changed by
 * a command/timer callback. Level 2 is the intentional fade endpoint. */
/* A fully shown screen, in the units of the fade that draw expressions read. */
#define CFW_EXPR_FADE_ONE 1024u

/* The timer period while a fade is running. A command is only picked up at the
 * next tick, so its fade starts up to this long after it arrives; Faceclaw
 * allows for that when it holds a frame back until a fade-out is over
 * (BRIGHTNESS_TICK_MS in GlassesSessionBrightness.kt: keep them in sync). */
#define BRIGHTNESS_TICK_MS 40u
/* The period once the level has settled: ownership only needs a slow check. */
#define BRIGHTNESS_IDLE_TICK_MS 1000u

/* The screen fade as a draw expression reads it (EX_FADE): `fade` is how far
 * the screen is into being visible, 0 (dark) to CFW_EXPR_FADE_ONE (shown), and
 * `moving` is set while a wake or sleep fade is under way. `read` is set once
 * any program has read it. */
typedef struct {
    uint16_t fade;
    uint8_t moving;
    uint8_t read;
} cfw_expression_fade;

typedef struct {
    uint32_t request, timer;
    uint32_t started, applied_level, present_epoch; /* applied_level=0 until written, or after observed power-off */
    uint16_t wait_epoch;
    uint16_t duration;
    uint8_t active, visible, waiting, level, from, target, desired;
    /* The wake or sleep fade as a fraction of being visible, for draw
     * expressions (EX_FADE): it runs from fade_from, in units of
     * 1/CFW_EXPR_FADE_ONE, to shown or dark over fade_duration ms. A change of
     * level while shown leaves it alone. */
    uint16_t fade_from, fade_duration;
    uint32_t fade_started;
} cfw_brightness_state;

/* brightness_service's answers to display_copy_hook. */
enum {
    BRIGHTNESS_SHOWN = 0,      /* draw as usual */
    BRIGHTNESS_HELD = 1,       /* the physical frame stays as it is: dark, or waiting for a wake's frame */
    BRIGHTNESS_FADING_OUT = 2, /* fading out: no new frame, but the one on screen may still be replayed */
};
static int brightness_control(const uint8_t *src, uint32_t size);
static int brightness_service(void);
static void brightness_cleanup(void);
/* Sample the fade for the frame about to be drawn (into draw_fade). For the
 * presenter, while the display task waits in its hook. */
static void brightness_fade_sample(void);
