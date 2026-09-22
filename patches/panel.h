#pragma once
#include <stdint.h>
#define CFW_PANEL_REQUEST_SIZE 12u
#define CFW_PANEL_REPLY_MAX 48u
/* Fixed requests: mode, id LE16, targets, op, length, address LE32, value LE16.
 * See docs/panel-diagnostics.md. State is published under the image mutex/gate;
 * hardware and result-cache access belong exclusively to the display task. */
typedef struct {
    volatile uint8_t pending;
    uint8_t origin, request[CFW_PANEL_REQUEST_SIZE];
    uint8_t cached_request[CFW_PANEL_REQUEST_SIZE], cached_origin;
    uint8_t result[CFW_PANEL_REPLY_MAX], result_length;
    uint8_t pattern; /* 0 = app, 1..16 = uniform codes, 17 ramp, 18 fixed patch */
    uint8_t saved, saved_sr3, baseline[7]; /* SR1, SR2, LUM[2], current, SR3, reserved */
    uint32_t register_deadline, pattern_deadline, watchdog;
} cfw_panel_state;
static int panel_queue(const uint8_t *src, uint32_t size, uint8_t origin);
static void panel_service(void);
static void panel_emit(uint8_t origin, uint16_t id, const uint8_t *data, uint8_t size);
