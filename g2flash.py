#!/usr/bin/env python3
"""
G2 firmware flasher — reimplements the official app's BLE flash protocol.

Two transports are supported, selected by the connection string passed on the
command line:

  g2://droidbridge?phone=<host>&port=<port>&token=<tok>&left=<mac>&right=<mac>
      Drive the glasses through a bonded phone running DroidBridge (lets us reuse
      a phone that is already paired/bonded with the glasses).

  g2://local?left=<addr>&right=<addr>&addressType=[public|random]
      Drive the glasses directly from this machine's Bluetooth radio via bleak.
      addressType=public is a normal MAC ("D0:7A:47:82:09:67"); addressType=
      random is the macOS / CoreBluetooth peripheral-UUID style. (Local mode
      needs `pip install bleak`.)

Protocol:
  transport: aa21 envelope  aa 21 seq len totFrags fragIdx sid flag <pb> crc16LE(last frag)
             CRC-16/CCITT-FALSE over the concatenated pb; chunkSize=232.
  DATA channel (svc e1001: write e0001 / notify e0002), sid byte = message type:
     sid 0xc0 control body=<opcode>[..]crc16: 0x00 begin, 0x01+128B EVENOTA
        subheader (FILE_CHECK), 0x02 data-block marker, 0x03 end.
     sid 0xc1 data: 4096-byte payload block (18 frags), CRC-16 on last.
     each write acked on notify as [opcode,status]; status 0 = OK else NAK.
  per-component CRC32C (MSB-first, init0, xorout0) is in the subheader, verified
  by the glasses on END.
  CTRL/EvenHub channel (svc e5450: write e5401 / notify e5402) carries the sid
  0x80 heartbeat ~15s — keep it alive during the transfer.

--stop-before gates the stages so a dry-run can't run past where we intend to stop.
"""
import time, json, struct, threading, queue, asyncio, sys, argparse, re, os, random, zlib, tempfile
import urllib.request, urllib.parse

# Debug fault injection (env-driven): force a single failure on component
# G2_FAULT_COMPONENT, block G2_FAULT_BLOCK, mode G2_FAULT_MODE ('frag'|'ack').
FAULT_COMPONENT = int(os.environ.get('G2_FAULT_COMPONENT','-1'))
FAULT_BLOCK     = int(os.environ.get('G2_FAULT_BLOCK','-1'))
FAULT_MODE      = os.environ.get('G2_FAULT_MODE','')
# G2_LOSS_RATE: probability of silently dropping each data-channel write (debug),
# to simulate a lossy write-without-response link across a whole transfer.
LOSS_RATE       = float(os.environ.get('G2_LOSS_RATE','0'))
_fault_fired    = [False]

# channel = (service uuid, write char, notify char). Mapping by handle order:
DATA = ("00002760-08c2-11e1-9073-0e8ac72e1001",   # firmware data svc (handles 0x082x)
        "00002760-08c2-11e1-9073-0e8ac72e0001",   # write  0x0822
        "00002760-08c2-11e1-9073-0e8ac72e0002")   # notify 0x0824
CTRL = ("00002760-08c2-11e1-9073-0e8ac72e5450",   # EvenHub/heartbeat svc (handles 0x084x)
        "00002760-08c2-11e1-9073-0e8ac72e5401",   # write  0x0842
        "00002760-08c2-11e1-9073-0e8ac72e5402")   # notify 0x0844

# Expected firmware layout: exactly 5 segments, one of which is the main image.
EXPECTED_SEGMENTS = 5
REQUIRED_SEGMENT = "ota/s200_firmware_ota.bin"

# --- main-app MRAM size ceiling (see memory: g2-firmware-size-limit) ---------
# The bootloader programs the main app into internal MRAM with NO bounds check:
# destination = preamble[0x14] and length = preamble[0]&0xFFFFFF are taken
# verbatim from the image and fed to unbounded erase+program loops. An image
# whose programmed end runs past the OTA flag / end of MRAM does NOT get
# rejected -- it overruns and (past MRAM end) deterministically bricks the lens
# into a bootloader-only, SWD-recovery state. So the ONLY guard is here.
APP_LOAD_ADDR = 0x00438000    # where the bootloader programs the main app (XIP)
MRAM_END      = 0x00800000    # end of Apollo510b 4 MB internal MRAM
OTA_FLAG_ADDR = 0x007FE000    # OTA magic word (last 8 KB) -- app must stay below
APP_PREAMBLE  = 0x20          # 32-byte preamble; bootloader programs payload[0x20:]
# Conservative ceiling: leave the top ~56 KB below the flag for BLE-bond/KV NV.
APP_MAX_END   = 0x007F0000

# how far to go: 'discover' | 'heartbeat' | 'file_check' | 'flash' | 'done'
STAGES = ["discover", "heartbeat", "file_check", "flash", "done"]
def allowed(stage, stop_before): return STAGES.index(stage) < STAGES.index(stop_before)

# ---- transfer tuning ----
# The c0/c1 OTA path has NO block index and NO dedup (firmware FUN_004dc14c): a
# `c0 op 0x02` marker resets the receive offset, c1 fragments accumulate by arrival,
# and the flash-writer advances ONE block per accepted block. Re-sending a block
# that was already written therefore double-advances the flash offset and corrupts
# everything after it -> the per-component END check (status 7 = CHECK_FAIL) fails.
# So: only resend in place on an explicit NAK (firmware rejected it, did not advance);
# on a bare ack-timeout (ambiguous: ack may just be lost on the bridge hop) abandon
# the attempt and re-flash the WHOLE component, which restarts cleanly from FILE_CHECK.
BLOCK_ACK_TIMEOUT = 4      # s to wait for a 4 KB block ack. A healthy block acks in
                           # well under 1s; if a marker/last-frag was dropped the block
                           # never completes and NO ack ever comes, so a long wait just
                           # sits in dead air until the BLE link's supervision timeout
                           # drops the connection. Fail fast and re-flash the component.
BLOCK_NAK_RETRIES = 3      # in-place resends on an explicit NAK (safe: firmware did not advance)
COMPONENT_RETRIES = 3      # whole-component re-flash attempts on END check-fail / ack-timeout

# eOTATransmitRsp (ota_transmit.proto) — ack status byte meanings.
OTA_RSP = {0:"SUCCESS",1:"HEADER_ERR",2:"PATH_ERR",3:"CRC_ERR",4:"TIMEOUT",5:"NO_RESOURCES",
           6:"FLASH_WRITE_ERR",7:"CHECK_FAIL",8:"UPDATING",9:"SYS_RESTART",10:"FAIL"}
def rsp_name(s): return OTA_RSP.get(s, f"0x{s:02x}")
# Normal END (RESULT_CHECK) acks: SUCCESS, plus UPDATING (8) which is what every
# component actually returns on a good flash, plus SYS_RESTART (9). 7=CHECK_FAIL is
# the real "bytes in flash don't match the component CRC" failure.
END_OK = {0, 8, 9}

# ---------------- framing (validated byte-for-byte vs capture) ----------------
def crc16(d):
    c=0xffff
    for b in d:
        c^=b<<8
        for _ in range(8): c=((c<<1)^0x1021)&0xffff if c&0x8000 else (c<<1)&0xffff
    return bytes([c&0xff,(c>>8)&0xff])
def crc32c_msb(buf,_t=[]):
    if not _t:
        for b in range(256):
            c=b<<24
            for _ in range(8): c=((c<<1)^0x1edc6f41)&0xffffffff if c&0x80000000 else (c<<1)&0xffffffff
            _t.append(c)
    crc=0
    for byte in buf: crc=((crc<<8)&0xffffffff)^_t[((crc>>24)^byte)&0xff]
    return crc
CHUNK=232
_seq=[0]
_seq_lock=threading.Lock()    # heartbeat thread + data path both call _nextseq()
def _reset_seq():
    with _seq_lock: _seq[0]=0
def _nextseq():
    with _seq_lock:
        _seq[0]=(_seq[0]+1)&0xff
        return _seq[0]
def frames(sid,pb,flag=0x00,seq=None):
    # seq=None -> allocate a fresh transport seq; pass an explicit seq to share one
    # across a marker+block pair (matches the official app).
    if seq is None: seq=_nextseq()
    body=pb+crc16(pb); tot=max(1,-(-len(body)//CHUNK)); out=[];off=0
    for i in range(tot):
        ch=body[off:off+CHUNK];off+=len(ch)
        out.append(bytes([0xaa,0x21,seq,len(ch),tot,i+1,sid,flag])+ch)
    return out
def ctrl_frames(op,data=b'',seq=None): return frames(0xc0,bytes([op])+data,seq=seq)
def data_frames(block,seq=None):        return frames(0xc1,block,seq=seq)

# ---------------- firmware parsing / validation ----------------
def parse_firmware_segments(img):
    """Unpack the OTA container into its component segments. Returns a list of
    dicts: {eid, off, size, crc, sub(128B subheader), ps(payload size), fn(name)}."""
    if len(img) < 0x40:
        raise ValueError("file is too small to be a firmware image")
    n = struct.unpack_from('<I', img, 8)[0]
    if not (0 < n <= 64):
        raise ValueError(f"implausible component count {n} (corrupt header?)")
    segs = []
    for i in range(n):
        eid, off, size, crc = struct.unpack_from('<IIII', img, 0x40 + i*16)
        sub = img[off:off+128]
        if len(sub) < 128:
            raise ValueError(f"segment {i} subheader runs past end of file")
        ps = struct.unpack_from('<I', sub, 8)[0]
        fn = sub[48:128].split(b'\0')[0].decode('latin1')
        segs.append({'eid':eid,'off':off,'size':size,'crc':crc,'sub':sub,'ps':ps,'fn':fn})
    return segs

def validate_firmware(img):
    """Sanity-check that the file looks like a flashable G2 firmware image. Returns
    the parsed segments on success, raises ValueError describing the problem."""
    segs = parse_firmware_segments(img)
    names = [s['fn'] for s in segs]
    if len(segs) != 5 and len(segs) != 6:
        raise ValueError(
            f"expected 5-6 segments, found {len(segs)}: {names}")
    if REQUIRED_SEGMENT not in names:
        raise ValueError(
            f"required segment {REQUIRED_SEGMENT!r} not found; segments are: {names}")
    # Each component stores a CRC32C of its payload in the TOC and the sub-header
    # echo (sub[12:16]); the glasses verify it on END (status 7 = CHECK_FAIL). The
    # flasher streams the sub-header verbatim, so a stale stored CRC (image patched
    # without recomputing checksums) guarantees a status-7 failure after a full,
    # slow transfer. Catch it here instead.
    for i, s in enumerate(segs):
        payload = img[s['off']+128:s['off']+128+s['ps']]
        calc = crc32c_msb(payload)
        sub_crc = struct.unpack_from('<I', s['sub'], 12)[0]
        if calc != s['crc'] or calc != sub_crc:
            raise ValueError(
                f"segment {i} ({s['fn']}) stored CRC32C is stale: payload computes "
                f"{calc:08x} but TOC={s['crc']:08x} sub={sub_crc:08x}. The image was "
                "modified without recomputing checksums — re-run the patch tool's "
                "checksum fixup (and the mainApp internal preamble CRC32) before flashing.")
    check_mainapp_fits_mram(img, segs)
    return segs

def check_mainapp_fits_mram(img, segs):
    """Guard against an enlarged main-app image overrunning internal MRAM.

    The bootloader programs the main app with NO size/destination bound (dst and
    length come straight from the 32-byte preamble). If the programmed region
    reaches the OTA flag (0x7FE000) it clobbers it + the BLE-bond/KV NV band; if
    it reaches the end of MRAM (0x800000) the write faults mid-erase and the lens
    is left in a permanent, BLE-unrecoverable bootloop (SWD-only recovery). This
    is the only place that can catch it, so fail loudly here.

    Also cross-checks the two fields the bootloader actually trusts:
      preamble[0x14] must be the load address 0x438000, and
      preamble[0]&0xFFFFFF (the length it programs) must equal the staged payload
      size ps -- if you enlarge the image you MUST bump the preamble length too,
      or the bootloader will program a different byte count than was streamed.
    """
    s = next((x for x in segs if x['fn'] == REQUIRED_SEGMENT), None)
    if s is None:                       # already caught above, but be defensive
        return
    ps = s['ps']
    pre = img[s['off']+128:s['off']+128+APP_PREAMBLE]
    if len(pre) < APP_PREAMBLE:
        raise ValueError("main-app payload is smaller than its 32-byte preamble")
    load_addr = struct.unpack_from('<I', pre, 0x14)[0]
    pre_len   = struct.unpack_from('<I', pre, 0)[0] & 0xFFFFFF
    if load_addr != APP_LOAD_ADDR:
        raise ValueError(
            f"main-app preamble load address is 0x{load_addr:08x}, expected "
            f"0x{APP_LOAD_ADDR:08x}. The bootloader programs to THIS address with "
            "no check — flashing would write the app to the wrong MRAM location.")
    if pre_len != ps:
        raise ValueError(
            f"main-app preamble length (0x{pre_len:x} = {pre_len} B) != staged "
            f"payload size ps (0x{ps:x} = {ps} B). The bootloader erases/programs "
            "preamble-length bytes but the flasher streams ps bytes; a mismatch "
            "truncates or overruns. If you resized the image, set preamble[0] "
            "low-24 to the new size and re-run --recompute-checksums.")
    prog_end = APP_LOAD_ADDR + ps - APP_PREAMBLE   # bootloader writes payload[0x20:]
    if prog_end > APP_MAX_END:
        over = prog_end - APP_MAX_END
        raise ValueError(
            f"main-app is too large: programmed region ends at 0x{prog_end:08x}, "
            f"{over} B ({over/1024:.1f} KB) past the safe ceiling 0x{APP_MAX_END:08x}. "
            f"MRAM app window is 0x{APP_LOAD_ADDR:08x}..0x{OTA_FLAG_ADDR:08x} (OTA flag); "
            f"end of MRAM is 0x{MRAM_END:08x}. The bootloader does NOT bounds-check "
            "this, so flashing risks clobbering the OTA flag / NV or bricking the lens "
            "(SWD-only recovery). Shrink the image or, if you have verified the exact "
            "NV base, raise APP_MAX_END deliberately.")
    headroom = APP_MAX_END - prog_end
    if DEBUG:
        print(f"    [main-app fits: ends 0x{prog_end:08x}, "
              f"{headroom} B ({headroom/1024:.0f} KB) under 0x{APP_MAX_END:08x}]")

def recompute_checksums(path):
    """Rewrite an EVENOTA image's stored checksums in place to match its current
    payloads — use after a length-preserving binary patch. Fixes each component's
    CRC32C (TOC entry @0x40+i*16+12 AND sub-header echo @off+0x0C) and the mainApp
    internal preamble CRC32 (zlib over payload[8:] @ payload+4). Writes atomically.
    Returns the number of fields changed."""
    with open(path,'rb') as fh: data=bytearray(fh.read())
    segs=parse_firmware_segments(bytes(data))   # validates structure / offsets
    changed=0
    for i,s in enumerate(segs):
        off=s['off']; ps=s['ps']; fn=s['fn']
        # The mainApp internal preamble CRC32 lives INSIDE the payload, so fix it
        # before computing the component CRC32C (which covers the whole payload).
        if fn.endswith('s200_firmware_ota.bin'):
            pre=zlib.crc32(bytes(data[off+128+8:off+128+ps]))&0xffffffff
            old=struct.unpack_from('<I',data,off+128+4)[0]
            if old!=pre:
                struct.pack_into('<I',data,off+128+4,pre); changed+=1
                print(f"  [{i}] {fn}: preamble crc32 {old:08x} -> {pre:08x}")
        payload=bytes(data[off+128:off+128+ps])
        crc=crc32c_msb(payload)
        old_toc=struct.unpack_from('<I',data,0x40+i*16+12)[0]
        old_sub=struct.unpack_from('<I',data,off+12)[0]
        if old_toc!=crc: struct.pack_into('<I',data,0x40+i*16+12,crc); changed+=1
        if old_sub!=crc: struct.pack_into('<I',data,off+12,crc); changed+=1
        if old_toc!=crc or old_sub!=crc:
            print(f"  [{i}] {fn}: component crc32c -> {crc:08x} (was TOC={old_toc:08x} sub={old_sub:08x})")
        else:
            print(f"  [{i}] {fn}: component crc32c {crc:08x} (ok)")
    if not changed:
        print(f"{path}: already consistent, no changes"); return 0
    st=os.stat(path); d=os.path.dirname(os.path.abspath(path))
    fd,tmp=tempfile.mkstemp(dir=d, prefix='.g2ck_')
    try:
        with os.fdopen(fd,'wb') as fh: fh.write(data)
        os.chmod(tmp, st.st_mode)
        os.replace(tmp, path)
    except BaseException:
        try: os.unlink(tmp)
        except OSError: pass
        raise
    print(f"updated {path}: {changed} checksum field(s) rewritten")
    return changed

# ---------------- DroidBridge client ----------------
class Bridge:
    def __init__(self, base, token):
        self.base=base.rstrip('/'); self.token=token
        self.notes=queue.Queue()      # (char_uuid_lower, bytes)
        self._stop=False
    def _req(self,path,obj=None,method=None):
        data=json.dumps(obj).encode() if obj is not None else None
        r=urllib.request.Request(self.base+path,data=data,method=method or ('POST' if data else 'GET'))
        if self.token: r.add_header('Authorization','Bearer '+self.token)
        if data: r.add_header('Content-Type','application/json')
        with urllib.request.urlopen(r,timeout=30) as resp: return resp.read()
    def status(self): return json.loads(self._req('/status'))
    def connect(self,a): return self._req('/connect',{"address":a})
    def discover(self,a): return self._req('/discover',{"address":a})
    def services(self,a): return json.loads(self._req('/services/'+a))
    def notify(self,a,svc,ch,en=True): return self._req('/notify',{"address":a,"service":svc,"characteristic":ch,"enable":en})
    def write(self,a,svc,ch,hexdata,wtype=1): return self._req('/write',{"address":a,"service":svc,"characteristic":ch,"data":hexdata,"writeType":wtype})
    def start_ws(self):
        import websocket
        self.ws_open=False
        def on_open(ws): self.ws_open=True
        def on_close(ws,*a): self.ws_open=False
        def on_msg(ws,msg):
            try:
                m=json.loads(msg)
                if m.get('type')=='notification':
                    d=m['data']; self.notes.put((d.get('characteristic','').lower(), bytes.fromhex(d.get('data',''))))
            except Exception: pass
        hdr=['Authorization: Bearer '+self.token] if self.token else None
        self.ws=websocket.WebSocketApp(self.base.replace('http','ws'),header=hdr,on_open=on_open,on_close=on_close,on_message=on_msg)
        threading.Thread(target=lambda: self.ws.run_forever(reconnect=3),daemon=True).start()
        def ka():
            while not self._stop:
                try: self.ws.send("ping")
                except Exception: pass
                time.sleep(1.5)
        threading.Thread(target=ka,daemon=True).start()

# ---------------- transports ----------------
# A transport is bound to a single lens address and exposes a uniform surface the
# flash routine drives: connect / discover / set_notify / write, plus a `notes`
# queue of (char_uuid_lower, bytes) notifications.

class DroidBridgeTransport:
    """Per-lens view onto a shared DroidBridge. Notifications arrive on the
    bridge's single websocket; since lenses are flashed one at a time, the shared
    queue is unambiguous."""
    def __init__(self, bridge, address):
        self.br=bridge; self.address=address; self.notes=bridge.notes
    def status(self):
        try: return f"droidbridge {self.br.base}: {self.br.status()}"
        except Exception as e: return f"droidbridge {self.br.base}: status failed ({e})"
    def connect(self):
        # fresh GATT state: stale notify/connection state makes begin time out
        try: self.br._req('/disconnect',{"address":self.address})
        except Exception: pass
        time.sleep(2)
        self.br.connect(self.address); time.sleep(2)
    def discover(self):
        self.br.discover(self.address)
        for _ in range(20):
            try:
                s=self.br.services(self.address)
                if s and s.get('services'): return True
            except Exception: pass
            time.sleep(1)
        return False
    def set_notify(self, svc, ch, enable=True):
        self.br.notify(self.address, svc, ch, enable)
    def write(self, svc, ch, hexdata, wtype=1):
        if LOSS_RATE>0 and ch==DATA[1] and random.random()<LOSS_RATE:
            return  # debug: simulate a dropped write-without-response fragment
        self.br.write(self.address, svc, ch, hexdata, wtype)
    def close(self):
        try: self.br._req('/disconnect',{"address":self.address})
        except Exception: pass

# G2 arms advertise as "Even G#_<serial>_<L|R>_<last-3-MAC-bytes>", e.g.
# "Even G2_32_L_693CCB" for MAC EC:D7:82:69:3C:CB. Groups: (serial, side, mactail).
G2_NAME_RE = re.compile(r'(?:even\s+)?G\d+_(\d+)_([LR])_([0-9a-fA-F]{6})', re.I)
SCAN_TIMEOUT = 20   # s to scan before giving up on finding an arm

def _norm_addr(s): return re.sub(r'[^0-9a-f]', '', (s or '').lower())

def match_scanned_device(devs, address, side):
    """Pick a scanned arm for the requested address/side. `devs` is bleak's
    discover(return_adv=True) dict {addr: (BLEDevice, AdvertisementData)}.

    macOS/CoreBluetooth never exposes BLE MACs (scanned addresses are random
    per-host UUIDs), so a MAC in the connection string can't match by address.
    But the advertised name carries the last 3 MAC bytes, so we match the MAC tail
    there. On Linux/BlueZ (and for UUID connection strings) the exact-address path
    matches directly. Returns a BLEDevice or None."""
    want=_norm_addr(address); want_tail=want[-6:]
    side_letter={'left':'L','right':'R'}.get(side or '')
    def name_of(d,adv): return adv.local_name or d.name or ""
    # 1) exact address match (Linux MAC, or macOS UUID connection strings)
    for a,(d,adv) in devs.items():
        if _norm_addr(a)==want: return d
    # 2) MAC tail embedded in the advertised name (the macOS case for a MAC string)
    for a,(d,adv) in devs.items():
        m=G2_NAME_RE.search(name_of(d,adv))
        if m and m.group(3).lower()==want_tail and (not side_letter or m.group(2).upper()==side_letter):
            return d
    # 3) last resort: exactly one advertised arm on the requested side
    if side_letter:
        cands=[d for a,(d,adv) in devs.items()
               if (lambda m: m and m.group(2).upper()==side_letter)(G2_NAME_RE.search(name_of(d,adv)))]
        if len(cands)==1: return cands[0]
    return None

class LocalBleTransport:
    """Direct connection over this machine's Bluetooth radio via bleak. bleak is
    async, so we run an event loop on a background thread and bridge each call
    across with run_coroutine_threadsafe. We scan first and connect to the
    discovered BLEDevice rather than connecting by raw address string — that is
    the only reliable path on macOS/CoreBluetooth (and is how the TS examples
    work)."""
    def __init__(self, address, address_type=None, side=None):
        self.address=address; self.address_type=address_type; self.side=side
        self.notes=queue.Queue()
        self.client=None
        self._loop=asyncio.new_event_loop()
        self._thr=threading.Thread(target=self._run_loop, daemon=True)
        self._thr.start()
    def _run_loop(self):
        asyncio.set_event_loop(self._loop); self._loop.run_forever()
    def _call(self, coro): return asyncio.run_coroutine_threadsafe(coro, self._loop).result()
    def status(self):
        return f"local BLE {self.side or '?'} {self.address}" + (f" ({self.address_type})" if self.address_type else "")
    def connect(self):
        from bleak import BleakClient, BleakScanner
        async def _c():
            devs=await BleakScanner.discover(timeout=SCAN_TIMEOUT, return_adv=True)
            dev=match_scanned_device(devs, self.address, self.side)
            if dev is None:
                seen=[f"{(adv.local_name or d.name)!r}@{a}" for a,(d,adv) in devs.items()
                      if G2_NAME_RE.search(adv.local_name or d.name or "")]
                raise RuntimeError(
                    f"could not find G2 {self.side or ''} ({self.address}) in a {SCAN_TIMEOUT}s scan. "
                    f"G2 arms advertising now: {seen or 'none'}. "
                    "The arm must be powered on and NOT connected to the phone (close the Even app / "
                    "turn off the phone's Bluetooth) so it advertises for a direct connection.")
            self.client=BleakClient(dev)
            await self.client.connect()
        self._call(_c())
    def discover(self):
        # bleak performs service discovery during connect()
        return bool(self.client and self.client.is_connected)
    def _on_note(self, fallback_ch):
        fb=fallback_ch.lower()
        def cb(sender, data):
            try: u=sender.uuid.lower()
            except Exception: u=fb
            self.notes.put((u, bytes(data)))
        return cb
    def set_notify(self, svc, ch, enable=True):
        async def _n():
            if enable: await self.client.start_notify(ch, self._on_note(ch))
            else: await self.client.stop_notify(ch)
        self._call(_n())
    def write(self, svc, ch, hexdata, wtype=1):
        data=bytes.fromhex(hexdata)
        if LOSS_RATE>0 and ch==DATA[1] and random.random()<LOSS_RATE:
            return  # debug: simulate a dropped write-without-response fragment
        # Android writeType 1 == WRITE_TYPE_NO_RESPONSE; anything else => with response.
        response=(wtype!=1)
        async def _w(): await self.client.write_gatt_char(ch, data, response=response)
        self._call(_w())
    def close(self):
        async def _d():
            if self.client and self.client.is_connected:
                await self.client.disconnect()
        try: self._call(_d())
        except Exception: pass
        self._loop.call_soon_threadsafe(self._loop.stop)

# ---------------- ack handling ----------------
DEBUG=False
def parse_rx(frame):
    """unwrap an aa12 reply envelope -> (sid, pb). pb is [opcode,status,...]."""
    if len(frame)>=10 and frame[0]==0xaa and frame[1]==0x12:
        ln=frame[3]; sid=frame[6]; pb=frame[8:8+max(0,ln-2)]
        return sid, pb
    return None, b''
def wait_ack(tp, want_op, ch_uuid, timeout=8):
    ch_uuid=ch_uuid.lower(); deadline=time.time()+timeout
    while time.time()<deadline:
        try: ch,frame=tp.notes.get(timeout=max(0.1,deadline-time.time()))
        except queue.Empty: break
        sid,pb=parse_rx(frame)
        if DEBUG: print(f"    [rx ...{ch[-4:]} sid=0x{sid:02x} pb={pb.hex()}]" if sid is not None else f"    [rx ...{ch[-4:]} {frame.hex()}]")
        if ch==ch_uuid and len(pb)>=2 and pb[0]==want_op: return pb[1]
    raise TimeoutError(f"no ack op=0x{want_op:02x} on {ch_uuid}")

def send_data_msg(tp, frames_list, want_op):
    svc,wch,nch=DATA
    for f in frames_list: tp.write(svc,wch,f.hex(),1)
    return wait_ack(tp, want_op, nch)

def send_block(tp, blk, fault=None):
    """Send one 4 KB block as marker+data sharing a single envelope seq (matches the
    official app). Drains stale acks first so we match THIS block's ack. Returns the
    ack status; raises TimeoutError if no ack arrives within BLOCK_ACK_TIMEOUT.

    `fault` (debug only) injects a controlled failure to exercise recovery:
      'frag' — drop one data fragment so the firmware sees a bad block CRC -> NAK.
      'ack'  — send the block fully (firmware writes it) but swallow its ack and
               raise TimeoutError, simulating a lost ack on a written block."""
    seq=_nextseq()
    while not tp.notes.empty(): tp.notes.get()
    for f in ctrl_frames(0x02, seq=seq): tp.write(DATA[0],DATA[1],f.hex(),1)   # marker: reset rx offset
    dfr=data_frames(blk, seq=seq)
    if fault=='frag':
        drop=len(dfr)//2
        print(f"     [FAULT] dropping data frag {drop}/{len(dfr)} to force a block NAK")
        dfr=[f for k,f in enumerate(dfr) if k!=drop]
    for f in dfr: tp.write(DATA[0],DATA[1],f.hex(),1)                          # 4 KB payload
    if fault=='ack':
        print("     [FAULT] block sent; swallowing its ack to simulate ack-loss -> timeout")
        try: wait_ack(tp, 0x02, DATA[2], timeout=BLOCK_ACK_TIMEOUT)
        except TimeoutError: pass
        raise TimeoutError("injected ack-loss")
    return wait_ack(tp, 0x02, DATA[2], timeout=BLOCK_ACK_TIMEOUT)

def flash_component(tp, seg, img, stop_before, comp_index=-1):
    """One pass over a component: FILE_CHECK, all 4 KB blocks, END. Returns the END
    ack status, or None if stop_before halts us before the data phase. Raises on a
    block-level failure (a NAK that survives in-place resends, or an ack-timeout) so
    the caller can re-flash the whole component from a clean state."""
    sub=seg['sub']; ps=seg['ps']; fn=seg['fn']
    payload=img[seg['off']+128:seg['off']+128+ps]
    st=send_data_msg(tp, ctrl_frames(0x01, sub), 0x01)            # FILE_CHECK / OTA_TRANSMIT_INFORMATION
    if st: raise RuntimeError(f"FILE_CHECK rejected status={st} ({rsp_name(st)})")
    if not allowed("flash", stop_before):
        return None
    nb=-(-len(payload)//4096)
    resends=0
    for b in range(nb):
        blk=payload[b*4096:(b+1)*4096]
        for tries in range(BLOCK_NAK_RETRIES):
            fault=None
            if comp_index==FAULT_COMPONENT and b==FAULT_BLOCK and not _fault_fired[0]:
                _fault_fired[0]=True; fault=FAULT_MODE or None
            try:
                st=send_block(tp, blk, fault=fault)
            except TimeoutError:
                print(f"     block {b}/{nb} ACK TIMEOUT after {BLOCK_ACK_TIMEOUT}s")
                raise                                            # -> whole-component re-flash (clean reset)
            if st==0: break
            resends+=1
            print(f"     block {b}/{nb} NAK={st} ({rsp_name(st)}) resend {tries+1}/{BLOCK_NAK_RETRIES}")
        else:
            raise RuntimeError(f"block {b} NAK'd {BLOCK_NAK_RETRIES}x")
        if b%100==0 or b==nb-1: print(f"     {fn}: block {b+1}/{nb}")
    print(f"     {fn}: data phase done, {resends} block resend(s); sending END")
    return send_data_msg(tp, ctrl_frames(0x03), 0x03)            # END / OTA_TRANSMIT_RESULT_CHECK

def flash_component_with_retry(tp, i, seg, img, stop_before):
    """Flash a component, re-flashing the whole thing if END verification fails or a
    block ack times out. Returns 0 on success, None if stopped early, raises if it
    can't get a clean END within COMPONENT_RETRIES attempts."""
    fn=seg['fn']; ps=seg['ps']
    payload=img[seg['off']+128:seg['off']+128+ps]
    print(f"[{i}] {fn} ({ps}B crc32c=0x{crc32c_msb(payload):08x})")
    for attempt in range(COMPONENT_RETRIES):
        if attempt: print(f"   re-flash attempt {attempt+1}/{COMPONENT_RETRIES}")
        try:
            end_st=flash_component(tp, seg, img, stop_before, comp_index=i)
        except (TimeoutError, RuntimeError) as e:
            print(f"   block phase failed: {e}")
            end_st=-1
        if end_st is None: return None
        if end_st in END_OK:
            print(f"   END verify OK ({fn}) status={end_st} ({rsp_name(end_st)})")
            return 0
        if end_st>=0:
            print(f"   END verify FAILED status={end_st} ({rsp_name(end_st)})")
        while not tp.notes.empty(): tp.notes.get()    # settle before the next attempt
        time.sleep(1.5)
    raise RuntimeError(f"component {fn} failed after {COMPONENT_RETRIES} attempts")

# ---------------- flash one lens ----------------
def flash_lens(tp, img, segs, stop_before):
    _reset_seq()
    print("status:", tp.status())
    tp.connect()
    ok=tp.discover()
    print("discovery:", "ok" if ok else "FAILED")
    if not ok:
        raise RuntimeError("service discovery failed")
    if not allowed("heartbeat", stop_before):
        print(f"[stop-before={stop_before}] discovery only; no writes."); return

    # enable notifications on the data + ctrl notify chars (give CCCD time to take)
    tp.set_notify(DATA[0],DATA[2],True)
    tp.set_notify(CTRL[0],CTRL[2],True)
    time.sleep(2.5)
    while not tp.notes.empty(): tp.notes.get()
    if not allowed("file_check", stop_before):
        print(f"[stop-before={stop_before}] notify set up; stopping before FILE_CHECK."); return

    # ---- (gated) actual flash ----
    # heartbeat keepalive on CTRL channel during the transfer (~12s, like the app)
    hb_stop=threading.Event()
    def hb_loop():
        while not hb_stop.wait(12):
            try:
                for f in frames(0x80, bytes.fromhex("080e10266a00")): tp.write(CTRL[0],CTRL[1],f.hex(),1)
            except Exception: pass
    threading.Thread(target=hb_loop,daemon=True).start()

    N=len(segs)
    print(f"flashing {len(img)}B, {N} components")
    while not tp.notes.empty(): tp.notes.get()          # drain stale notifications
    bst=send_data_msg(tp, ctrl_frames(0x00), 0x00)
    print(f"begin ack {bst} ({rsp_name(bst)})")
    if bst not in END_OK: print(f"   WARNING: unexpected begin status {bst} ({rsp_name(bst)}); continuing")
    t_start=time.time()
    try:
        for i,seg in enumerate(segs):
            if flash_component_with_retry(tp, i, seg, img, stop_before) is None:
                print(f"[stop-before={stop_before}] FILE_CHECK acked; stopping before data blocks.")
                return
    finally:
        hb_stop.set()
    print(f"=== all {N} components verified in {time.time()-t_start:.0f}s; glasses should reboot into the new firmware ===")

# ---------------- connection string ----------------
def parse_connection_string(raw):
    """Parse g2://<method>?... into a dict. Methods: 'local', 'droidbridge'."""
    u=urllib.parse.urlparse(raw.strip())
    if u.scheme!='g2':
        raise ValueError(f"connection string must start with 'g2://', got {raw!r}")
    method=u.netloc.lower()
    q={k:v[0] for k,v in urllib.parse.parse_qs(u.query).items()}
    left=q.get('left'); right=q.get('right')
    if not left or not right:
        raise ValueError("connection string must include left= and right=")
    if method=='local':
        at=(q.get('addressType') or '').lower() or None
        if at not in (None,'public','random'):
            raise ValueError("addressType must be 'public' or 'random'")
        return {'method':'local','left':left,'right':right,'address_type':at}
    if method=='droidbridge':
        phone=q.get('phone'); port=q.get('port'); token=q.get('token','')
        if not phone or not port:
            raise ValueError("droidbridge connection string must include phone= and port=")
        return {'method':'droidbridge','left':left,'right':right,
                'base':f"http://{phone}:{port}",'token':token}
    raise ValueError(f"unknown connection method {method!r}; expected 'local' or 'droidbridge'")

# ---------------- warranty gate ----------------
WARRANTY_PHRASE="my warranty is void"
def confirm_warranty(skip):
    print("="*72)
    print("WARNING: flashing a custom firmware will VOID your G2's warranty and")
    print("carries a real risk of BRICKING the device. Proceed only if you")
    print("understand and accept that risk.")
    print("="*72)
    if skip:
        print('--my-warranty-is-void supplied; skipping interactive confirmation.')
        return
    try:
        resp=input(f'Type "{WARRANTY_PHRASE}" to continue: ')
    except (EOFError, KeyboardInterrupt):
        print("\nAborted."); sys.exit(1)
    if resp.strip()!=WARRANTY_PHRASE:
        print("Phrase did not match. Aborted."); sys.exit(1)

# ---------------- main ----------------
def main(argv=None):
    global DEBUG, COMPONENT_RETRIES, BLOCK_NAK_RETRIES
    p=argparse.ArgumentParser(description="Flash firmware onto Even Realities G2 glasses.")
    p.add_argument('-c','--connection',
                   help="connection string: g2://droidbridge?phone=..&port=..&token=..&left=..&right=.. "
                        "or g2://local?left=..&right=..&addressType=public|random")
    p.add_argument('-f','--firmware', help="path to the firmware image to flash")
    p.add_argument('--recompute-checksums', metavar='IMAGE',
                   help="rewrite IMAGE's stored checksums in place to match its payloads "
                        "(component CRC32C + mainApp preamble CRC32), then exit. Use after a "
                        "length-preserving binary patch; does not connect or flash.")
    p.add_argument('--lens', choices=['left','right','both'], default='both',
                   help="which lens to flash (default: both)")
    p.add_argument('--stop-before', choices=STAGES, default='done',
                   help="stop before this stage (dry-run gating; default: done = full flash)")
    p.add_argument('--my-warranty-is-void', action='store_true',
                   help="skip the interactive warranty confirmation (for automation)")
    p.add_argument('--component-retries', type=int, default=COMPONENT_RETRIES,
                   help=f"whole-component re-flash attempts on END fail/timeout (default {COMPONENT_RETRIES})")
    p.add_argument('--block-nak-retries', type=int, default=BLOCK_NAK_RETRIES,
                   help=f"in-place block resends on an explicit NAK (default {BLOCK_NAK_RETRIES})")
    p.add_argument('--debug', action='store_true', help="print received frames")
    args=p.parse_args(argv)

    DEBUG=args.debug
    COMPONENT_RETRIES=args.component_retries
    BLOCK_NAK_RETRIES=args.block_nak_retries

    if args.recompute_checksums:
        try:
            recompute_checksums(args.recompute_checksums)
        except (OSError, ValueError) as e:
            print(f"recompute-checksums failed: {e}"); sys.exit(1)
        return

    if not args.connection or not args.firmware:
        p.error("-c/--connection and -f/--firmware are required (unless --recompute-checksums)")

    try:
        conn=parse_connection_string(args.connection)
    except ValueError as e:
        p.error(str(e))

    try:
        with open(args.firmware,'rb') as fh: img=fh.read()
    except OSError as e:
        print(f"cannot read firmware: {e}"); sys.exit(1)
    try:
        segs=validate_firmware(img)
    except ValueError as e:
        print(f"firmware validation failed: {e}"); sys.exit(1)
    print(f"firmware ok: {args.firmware} ({len(img)}B, {len(segs)} segments: {[s['fn'] for s in segs]})")

    confirm_warranty(args.my_warranty_is_void)

    sides=['left','right'] if args.lens=='both' else [args.lens]

    bridge=None
    if conn['method']=='droidbridge':
        bridge=Bridge(conn['base'], conn['token']); bridge.start_ws(); time.sleep(2)

    failures=[]
    for side in sides:
        addr=conn[side]
        print(f"\n=== flashing {side} lens ({addr}) ===")
        if conn['method']=='local':
            tp=LocalBleTransport(addr, conn.get('address_type'), side=side)
        else:
            tp=DroidBridgeTransport(bridge, addr)
        try:
            flash_lens(tp, img, segs, args.stop_before)
        except Exception as e:
            print(f"!!! {side} lens flash failed: {e}")
            failures.append(side)
        finally:
            tp.close()

    if bridge is not None:
        bridge._stop=True

    if failures:
        print(f"\nFAILED lenses: {failures}"); sys.exit(1)
    print("\nall selected lenses completed.")

if __name__=="__main__":
    main()
