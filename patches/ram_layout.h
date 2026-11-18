#pragma once

/* RAM between the stock LVGL and primary heaps, firmware 2.3.0.24.
 *
 * Stock places three TLSF arenas back to back in one zero-initialized block:
 *
 *   LVGL / GPU pool  [0x2013519c, 0x2020219c)  0xcd000  not cached (MPU region 6)
 *   EvenHub          [0x2020219c, 0x2027299c)  0x70800
 *   primary          [0x2027299c, 0x2029f99c)  0x2d000
 *
 * patch_compress.py lowers the top of the LVGL arena and raises the bottom of
 * the EvenHub arena, and ends MPU region 6 at the new top of the LVGL arena.
 * The gap holds the two large CFW buffers, at fixed addresses, in cached RAM:
 *
 *   LVGL / GPU pool  [0x2013519c, 0x201bb19c)  0x86000  not cached
 *   (padding)        [0x201bb19c, 0x201bb1a0)  the MPU rounds the region's end
 *                                              up to a 32-byte boundary
 *   screen buffer    [0x201bb1a0, 0x201e09a0)  0x25800
 *   resource cache   [0x201e09a0, 0x202311a0)  0x50800
 *   (unused)         [0x202311a0, 0x2023199c)  0x7fc
 *   EvenHub          [0x2023199c, 0x2027299c)  0x41000
 *   primary          [0x2027299c, 0x2029f59c)  0x2cc00
 *   CFW context slot [0x2029f59c, 0x2029f99c)  see cfw_context.h
 *
 * The heap sizes are instruction immediates with eight significant bits, which
 * is why they are round numbers and 0x7fc bytes are left over. The stock
 * startup code zeroes the whole block on every boot.
 *
 * Every number here is repeated in patch_compress.py; tests/test_ram_layout.py
 * compares the two and checks them against the stock image. */
#define CFW_LVGL_ARENA           0x2013519cU
#define CFW_LVGL_ARENA_SIZE      0x00086000U
#define CFW_SCREEN_BUFFER_ADDR   0x201bb1a0U
#define CFW_SCREEN_BUFFER_SIZE   0x00025800U  /* 640 x 480, 4 bits per pixel */
#define CFW_RESOURCE_CACHE_ADDR  0x201e09a0U
#define CFW_RESOURCE_CACHE_BYTES 0x00050800U  /* 2 KiB id table + 320 KiB data */
#define CFW_EVENHUB_ARENA        0x2023199cU
#define CFW_EVENHUB_ARENA_SIZE   0x00041000U
#define CFW_PRIMARY_ARENA        0x2027299cU
#define CFW_PRIMARY_ARENA_SIZE   0x0002cc00U

_Static_assert(CFW_LVGL_ARENA + CFW_LVGL_ARENA_SIZE <= CFW_SCREEN_BUFFER_ADDR,
               "screen buffer overlaps the LVGL arena");
_Static_assert((CFW_SCREEN_BUFFER_ADDR & 31u) == 0 && (CFW_RESOURCE_CACHE_ADDR & 31u) == 0,
               "CFW buffers must start outside the uncached MPU region");
_Static_assert(CFW_SCREEN_BUFFER_ADDR + CFW_SCREEN_BUFFER_SIZE <= CFW_RESOURCE_CACHE_ADDR,
               "resource cache overlaps the screen buffer");
_Static_assert(CFW_RESOURCE_CACHE_ADDR + CFW_RESOURCE_CACHE_BYTES <= CFW_EVENHUB_ARENA,
               "EvenHub arena overlaps the resource cache");
_Static_assert(CFW_EVENHUB_ARENA + CFW_EVENHUB_ARENA_SIZE == CFW_PRIMARY_ARENA,
               "EvenHub arena must end where the primary arena starts");
