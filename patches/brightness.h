#pragma once
#include <stdint.h>

/* The command task owns timer creation and the inbound PRESENT epoch, and
 * publishes one immutable request word. Fade state belongs to the display task. No panel register or persistent setting is changed by
 * a command/timer callback. Level 2 is the intentional fade endpoint. */
typedef struct {
    uint32_t request, timer;
    uint32_t started, applied_level, present_epoch; /* applied_level=0 until written, or after observed power-off */
    uint16_t wait_epoch;
    uint16_t duration;
    uint8_t active, visible, waiting, level, from, target, desired;
} cfw_brightness_state;
static int brightness_control(const uint8_t *src, uint32_t size);
static int brightness_service(void);
static void brightness_cleanup(void);
