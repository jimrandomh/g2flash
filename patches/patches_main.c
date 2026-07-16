/*
 * Single translation unit for all injected CFW patch code.
 *
 * Historically the four patch sources below were each compiled and PLACED as a
 * separate relocatable blob, which meant they could not call one another and
 * patch_compress.py had to lay out four blobs at four base addresses (and special-
 * case extracting a single function out of decompress.c). Compiling them as ONE
 * translation unit instead yields a SINGLE blob: build.py's mini-linker resolves any
 * cross-file call/relocation, and every injected entry point is simply
 * base + its offset in the one blob.
 *
 * The four sources deliberately share no typedef / macro / function names, so a plain
 * #include chain compiles cleanly. Each keeps its own `#include <stdint.h>` (idempotent
 * via the standard header guard). Order is not significant — build.py looks functions
 * up by name — but decompress.c is first so the memcpy-ABI frag_write leads the blob.
 *
 * Being one translation unit is also what lets the injected code take the address of
 * its own functions (z_stream zalloc/zfree, the seq_tick osTimer callback) with plain
 * `&fn`: -fropi makes an intra-CU function address PC-relative and relocation-free, so
 * no load address is needed at build time. Splitting these back into separate CUs would
 * turn those into absolute relocations that build.py rejects.
 */
#include "decompress.c"
#include "zlib_glue.c"
#include "settings_ext.c"
#include "gesture_fwd.c"
