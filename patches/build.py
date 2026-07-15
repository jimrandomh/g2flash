#!/usr/bin/env python3
"""
Compile a C source to position-independent Thumb-2 machine code for the G2
mainapp core (ARMv7E-M, Cortex-M-class), verify it has NO relocations and no
external calls, and emit the raw .text bytes.

Usage:
  python3 build.py <src.c> [-Dname=val ...]           # human report + obj/<stem>.text.bin
  python3 build.py <src.c> [-Dname=val ...] --json     # machine-readable JSON to stdout

Build intermediates (<stem>.o, <stem>.text.bin) are written to g2flash/obj/ (created
on demand), not next to the sources.

Human mode prints, per exported function, its offset/size and the raw bytes (hex)
and writes obj/<stem>.text.bin. JSON mode prints a single object:

  {
    "src": "zlib_glue.c",
    "text_len": 1394,
    "text": "<hex of the full .text section>",
    "functions": [{"name": "...", "offset": 0, "size": 14, "bytes": "<hex>"}, ...]
  }

so patch_compress.py can pull the exact bytes it injects straight from the build
instead of carrying pasted hex. --json has no side effects beyond the obj/<stem>.o the
compiler emits (it does NOT write obj/<stem>.text.bin).

Self-containedness is enforced in both modes. build.py acts as a mini-linker over
the emitted blob and resolves two relocation families in place:

  * Intra-.text branches (R_ARM_THM_CALL / R_ARM_THM_JUMP24 to a symbol defined in
    .text): the BL/B.W displacement is rewritten, so injected functions can call
    each other by name (incl. from inline asm) without an "everything must be
    static" restriction.

  * PC-relative read-only-data references (R_ARM_THM_MOVW_PREL_NC / MOVT_PREL,
    emitted under -fropi): string literals and other read-only constants. The
    referenced .rodata* sections are appended to the blob right after .text and
    the movw/movt immediate pair is fixed up so the runtime `add rX, pc` lands on
    the datum. Because these are PC-relative, the fixup depends only on the datum's
    offset WITHIN the blob, so the result stays position-independent regardless of
    where the blob is later loaded -- exactly like the branch case. This is why we
    compile with -fropi: absolute (MOVW_ABS/MOVT_ABS) rodata refs would need the
    final load address, which only patch_compress.py knows.

The emitted bytes are therefore .text followed by any referenced .rodata; the
returned/reported `text`/`text_len` cover the whole blob and function offsets stay
blob-relative (i.e. .text-relative, since .text is first). `rodata_len` reports how
many trailing bytes are data -- callers that extract a SINGLE function's bytes
(rather than appending the whole blob) must assert rodata_len == 0, since a lone
function carries no rodata with it.

Any OTHER relocation -- an external/undefined branch target, an absolute rodata
ref, or a relocation inside the rodata itself (e.g. an array of pointers to string
literals, which needs data-to-data fixups) -- is still a hard error, because PIC
injection has no linker to fix absolute addresses up (firmware entry points must be
called via absolute-constant function pointers instead).
"""
import sys, os, struct, subprocess, json

# Build intermediates (.o, .text.bin) go in g2flash/obj (a sibling of this patches/
# dir), created on demand, rather than cluttering the source tree next to the .c files.
OBJ_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "obj")

def obj_path(src, suffix):
    """Path for a build intermediate of `src` (by basename) inside OBJ_DIR."""
    os.makedirs(OBJ_DIR, exist_ok=True)
    stem = os.path.basename(src).rsplit(".", 1)[0]
    return os.path.join(OBJ_DIR, stem + suffix)

R_ARM_THM_CALL         = 10   # BL / BLX  (Thumb-2, 32-bit)
R_ARM_THM_JUMP24       = 30   # B.W       (Thumb-2, 32-bit)
R_ARM_THM_MOVW_PREL_NC = 49   # movw rX, #:lower16:(sym - .)   (PC-relative, -fropi)
R_ARM_THM_MOVT_PREL    = 50   # movt rX, #:upper16:(sym - .)   (PC-relative, -fropi)

def resolve_thumb_branch(tbytes, off, target):
    """Rewrite the Thumb-2 BL/B.W at tbytes[off:off+4] to branch to .text offset
    `target`, preserving the BL-vs-B.W opcode bits. Both are PC-relative to off+4."""
    hw2_old = tbytes[off + 2] | (tbytes[off + 3] << 8)
    disp = target - (off + 4)
    if disp % 2 or not (-(1 << 24) <= disp < (1 << 24)):
        raise BuildError(f"branch at {off:#x} -> {target:#x} out of Thumb range (disp {disp})")
    imm = (disp >> 1) & 0xFFFFFF
    S     = (imm >> 23) & 1
    i1    = (imm >> 22) & 1
    i2    = (imm >> 21) & 1
    imm10 = (imm >> 11) & 0x3FF
    imm11 = imm & 0x7FF
    j1 = (~(i1 ^ S)) & 1
    j2 = (~(i2 ^ S)) & 1
    hw1 = 0xF000 | (S << 10) | imm10
    hw2 = (hw2_old & 0xD000) | (j1 << 13) | (j2 << 11) | imm11   # keep bits 15/14(type)/12
    tbytes[off:off + 4] = bytes([hw1 & 0xFF, hw1 >> 8, hw2 & 0xFF, hw2 >> 8])

def _thumb_movwt_get_imm(hw1, hw2):
    """Extract the 16-bit immediate encoded in a Thumb-2 movw/movt (T3) pair."""
    imm4 = hw1 & 0xF
    i    = (hw1 >> 10) & 1
    imm3 = (hw2 >> 12) & 7
    imm8 = hw2 & 0xFF
    return (imm4 << 12) | (i << 11) | (imm3 << 8) | imm8

def _thumb_movwt_set_imm(hw1, hw2, val):
    """Return (hw1, hw2) with the 16-bit immediate field replaced by `val`."""
    val &= 0xFFFF
    imm4 = (val >> 12) & 0xF
    i    = (val >> 11) & 1
    imm3 = (val >> 8) & 7
    imm8 = val & 0xFF
    hw1 = (hw1 & ~((1 << 10) | 0xF)) | (i << 10) | imm4
    hw2 = (hw2 & ~((7 << 12) | 0xFF)) | (imm3 << 12) | imm8
    return hw1, hw2

def resolve_movwt(blob, off, sym_addr, high):
    """Fix up the PC-relative Thumb movw (high=False) / movt (high=True) at
    blob[off:off+4] so the movw/movt pair materializes `sym_addr - P`, where the
    symbol and the instruction are both blob-relative (P == off). The addend is
    read in place (ELF REL form, sign-extended 16-bit). movw takes bits [15:0] of
    the result; movt takes bits [31:16]. Together with the `add rX, pc` the
    compiler emits, this reconstructs `sym_addr` at runtime, independent of load
    address (the relative distance is what's baked in)."""
    hw1 = blob[off] | (blob[off + 1] << 8)
    hw2 = blob[off + 2] | (blob[off + 3] << 8)
    addend = _thumb_movwt_get_imm(hw1, hw2)
    if addend & 0x8000:
        addend -= 0x10000
    result = (sym_addr + addend - off) & 0xFFFFFFFF
    val = (result >> 16) & 0xFFFF if high else result & 0xFFFF
    hw1, hw2 = _thumb_movwt_set_imm(hw1, hw2, val)
    blob[off:off + 4] = bytes([hw1 & 0xFF, hw1 >> 8, hw2 & 0xFF, hw2 >> 8])

CLANG = "clang"
CFLAGS = [
    "--target=thumbv7em-none-eabi", "-mthumb",
    "-O2", "-ffreestanding", "-fno-jump-tables", "-fomit-frame-pointer",
    "-fno-builtin", "-mno-unaligned-access",
    "-fno-unwind-tables", "-fno-asynchronous-unwind-tables",
    "-fropi",   # PC-relative rodata refs so string literals resolve in-blob (see module docstring)
    "-Wall", "-Wextra",
]

# ---- minimal ELF32 LE parser (section headers + symtab) ----
def parse_elf(path):
    d = open(path, "rb").read()
    assert d[:4] == b"\x7fELF" and d[4] == 1 and d[5] == 1, "not ELF32-LE"
    (e_shoff,) = struct.unpack_from("<I", d, 0x20)
    e_shentsize, e_shnum, e_shstrndx = struct.unpack_from("<HHH", d, 0x2e)
    secs = []
    for i in range(e_shnum):
        off = e_shoff + i * e_shentsize
        name, typ, flags, addr, offset, size, link, info, align, entsz = \
            struct.unpack_from("<IIIIIIIIII", d, off)
        secs.append(dict(name=name, type=typ, flags=flags, offset=offset,
                         size=size, link=link, info=info, align=align, entsize=entsz))
    shstr = secs[e_shstrndx]
    def sname(n):
        s = d[shstr["offset"] + n:]
        return s[:s.index(b"\0")].decode()
    for s in secs:
        s["sname"] = sname(s["name"])
    return d, secs

def section(secs, name):
    for s in secs:
        if s["sname"] == name:
            return s
    return None

class BuildError(Exception):
    pass

SHT_PROGBITS = 1
SHT_REL      = 9
SHF_WRITE    = 0x1
SHF_ALLOC    = 0x2
SHF_EXECINSTR = 0x4

def _is_rodata(sec):
    """True for a read-only allocated data section (rodata, string/constant pools):
    PROGBITS, ALLOC, neither writable nor executable. Name-agnostic so .rodata,
    .rodata.str1.1, .rodata.cst16, .rodata.* are all captured."""
    return (sec["type"] == SHT_PROGBITS
            and (sec["flags"] & SHF_ALLOC)
            and not (sec["flags"] & (SHF_WRITE | SHF_EXECINSTR)))

def compile_text(src, extra=()):
    """Compile `src` to Thumb-2 and return (blob, funcs, rodata_len). `blob` is the
    .text bytes followed by any referenced .rodata* (both relocation families
    resolved in place; see module docstring); `funcs` is a list of (name, offset,
    size) with offsets blob-relative (== .text-relative, since .text is first) and
    sizes resolved; `rodata_len` is the count of trailing data bytes. Raises
    BuildError on any relocation that can't be resolved position-independently or
    any reference to an external/undefined symbol. Sizes are resolved from st_size,
    falling back to the gap to the next function (or end of .text) when 0."""
    obj = obj_path(src, ".o")
    subprocess.run([CLANG, *CFLAGS, *extra, "-c", src, "-o", obj], check=True)

    d, secs = parse_elf(obj)
    text = section(secs, ".text")
    text_idx = secs.index(text)

    # Lay out the blob: .text first, then every referenced-able read-only data
    # section, each aligned to its own sh_addralign. `base[shndx]` maps a section
    # index to where it starts in the blob, so a symbol's blob address is
    # base[sym.shndx] + sym.value and a relocation site's blob offset is
    # base[target_section] + r_offset -- all blob-relative, hence load-independent.
    blob = bytearray(d[text["offset"]:text["offset"] + text["size"]])
    text_len = len(blob)
    base = {text_idx: 0}
    for i, s in enumerate(secs):
        if i == text_idx or not _is_rodata(s):
            continue
        align = max(s["align"], 1)
        while len(blob) % align:
            blob.append(0)
        base[i] = len(blob)
        blob += d[s["offset"]:s["offset"] + s["size"]]
    rodata_len = len(blob) - text_len

    # collect all symbols (name, value, size, type, section index)
    symtab = section(secs, ".symtab")
    strtab = secs[symtab["link"]]
    syms = []
    for i in range(symtab["size"] // 16):
        o = symtab["offset"] + i * 16
        st_name, st_value, st_size, st_info, st_other, st_shndx = \
            struct.unpack_from("<IIIBBH", d, o)
        nm = d[strtab["offset"] + st_name:]
        nm = nm[:nm.index(b"\0")].decode()
        syms.append(dict(name=nm, value=st_value, size=st_size,
                         typ=st_info & 0xf, shndx=st_shndx))

    # Resolve relocations against any section we laid into the blob. clang emits
    # ELF REL for ARM (`.rel.<sec>`, 8-byte entries, addend in-place); `.rela.*`
    # (12-byte) is tolerated for iteration. Two families are resolvable, both fully
    # PC-relative so the fixup is a blob-internal constant:
    #   * intra-.text BL/B.W  (R_ARM_THM_CALL / JUMP24 to a .text symbol)
    #   * PC-relative rodata refs (R_ARM_THM_MOVW_PREL_NC / MOVT_PREL, -fropi)
    # Anything else -- absolute rodata refs, data-to-data pointer relocs, external
    # branch targets -- is a hard error (no linker to bake in an absolute address).
    bad = []
    for rs in secs:
        if rs["type"] != SHT_REL and not rs["sname"].startswith(".rela"):
            continue
        target = rs["info"]                 # sh_info = section these relocs apply to
        if target not in base:              # e.g. .rel.ARM.exidx, .rel.debug_* -> ignore
            continue
        ent = 12 if rs["sname"].startswith(".rela") else 8
        for i in range(rs["size"] // ent):
            r_offset, r_info = struct.unpack_from("<II", d, rs["offset"] + i * ent)
            r_type = r_info & 0xff
            sym = syms[r_info >> 8]
            wpos = base[target] + r_offset  # blob offset of the site being patched
            if (r_type in (R_ARM_THM_CALL, R_ARM_THM_JUMP24)
                    and target == text_idx and sym["shndx"] == text_idx):
                resolve_thumb_branch(blob, wpos, sym["value"] & ~1)
            elif r_type in (R_ARM_THM_MOVW_PREL_NC, R_ARM_THM_MOVT_PREL) \
                    and sym["shndx"] in base:
                resolve_movwt(blob, wpos, base[sym["shndx"]] + sym["value"],
                              high=(r_type == R_ARM_THM_MOVT_PREL))
            else:
                bad.append(f"  {rs['sname']}+{r_offset:#x} type={r_type} -> "
                           f"{sym['name']!r} (shndx={sym['shndx']}); only intra-.text "
                           f"BL/B.W and PC-relative (-fropi) rodata refs are resolvable")
    if bad:
        raise BuildError(f"{src}: unresolvable relocation(s) — call firmware entry "
                         f"points via absolute-constant fn-ptrs (not by name), and keep "
                         f"read-only data free of pointers into other data:\n"
                         + "\n".join(bad))

    # collect STT_FUNC symbols for the report / patch_compress
    raw_funcs = [(s["name"], s["value"] & ~1, s["size"])
                 for s in syms if s["typ"] == 2 and s["shndx"] == text_idx]
    # resolve sizes: st_size, else gap to next function, else end of .text
    raw_funcs.sort(key=lambda x: x[1])
    funcs = []
    for i, (nm, val, sz) in enumerate(raw_funcs):
        if not sz:
            nxt = raw_funcs[i + 1][1] if i + 1 < len(raw_funcs) else text_len
            sz = nxt - val
        funcs.append((nm, val, sz))
    return bytes(blob), funcs, rodata_len

def build_dict(src, extra=()):
    blob, funcs, rodata_len = compile_text(src, extra)
    return {
        "src": src,
        "text_len": len(blob),
        "rodata_len": rodata_len,
        "text": blob.hex(),
        "functions": [
            {"name": nm, "offset": val, "size": sz, "bytes": blob[val:val + sz].hex()}
            for nm, val, sz in funcs
        ],
    }

def main():
    args = sys.argv[1:]
    as_json = "--json" in args
    args = [a for a in args if a != "--json"]
    extra = [a for a in args if a.startswith("-")]   # e.g. -DFOO=0x1234
    src = next(a for a in args if not a.startswith("-"))

    try:
        if as_json:
            print(json.dumps(build_dict(src, extra)))
            return
        blob, funcs, rodata_len = compile_text(src, extra)
    except BuildError as e:
        print("FAIL:", e)
        sys.exit(1)

    text_len = len(blob) - rodata_len
    rod = f", +{rodata_len} B rodata" if rodata_len else ""
    print(f"OK: {os.path.basename(src)} blob = {len(blob)} bytes (.text {text_len}{rod}), "
          f"relocs resolved, no external refs\n")
    for nm, val, sz in sorted(funcs, key=lambda x: x[1]):
        b = blob[val:val + sz]
        print(f"== {nm}  (text+{val:#x}, {sz} bytes) ==")
        print("bytes:", b.hex())
        print()

    text_bin = obj_path(src, ".text.bin")
    open(text_bin, "wb").write(blob)
    print(f"wrote {text_bin} ({len(blob)} bytes)")

if __name__ == "__main__":
    main()
