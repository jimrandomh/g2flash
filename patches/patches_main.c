/*
 * Single translation unit for all injected CFW patch code.
 *
 * Historically the patch sources below were each compiled and PLACED as a
 * separate relocatable blob, which meant they could not call one another and
 * patch_compress.py had to lay out several blobs at several base addresses. Compiling
 * them as ONE translation unit instead yields a SINGLE blob: build.py's mini-linker
 * resolves any cross-file call/relocation, and every injected entry point is simply
 * base + its offset in the one blob.
 *
 * The sources deliberately share no typedef / macro / function names, so a plain
 * #include chain compiles cleanly. Each keeps its own `#include <stdint.h>` (idempotent
 * via the standard header guard). Order is not significant — build.py looks functions
 * up by name.
 *
 * Being one translation unit is also what lets the injected code take the address of
 * its own functions (z_stream zalloc/zfree, the seq_tick osTimer callback) with plain
 * `&fn`: -fropi makes an intra-CU function address PC-relative and relocation-free, so
 * no load address is needed at build time. Splitting these back into separate CUs would
 * turn those into absolute relocations that build.py rejects.
 *
 * decompress.c (frag_write: the CompressMode-driven per-fragment 1bpp->4bpp expander)
 * was dropped in the 2.2.6.10 rebase — stock now uses CompressMode itself for RLE/LZ4,
 * and image_deferred in zlib_glue.c is the better decompressor. The file is kept in the
 * tree for reference but is deliberately NOT built.
 */
#include "zlib_glue.c"
#include "settings_ext.c"
#include "gesture_fwd.c"
