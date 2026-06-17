#!/usr/bin/env python3
"""
G2 firmware flasher — reimplements the official app's BLE flash protocol,
driving the glasses through the bonded phone via DroidBridge.

Protocol (reverse-engineered + validated byte-for-byte against a real flash):
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

STOP_BEFORE gates the stages so a dry-run can't run past where we intend to stop.
"""
import time, json, struct, threading, queue
import urllib.request

# ---------------- config ----------------
PHONE   = "http://192.168.1.71:8765"
TOKEN   = "Hohh6uuroo4GeiPh"
LEFT_LENS  = "EC:D7:82:69:3C:CB" #Left
RIGHT_LENS = "D0:7A:47:82:09:67" #Right
LENS = RIGHT_LENS

IMAGE   = "/Users/jbabcock/g2-flashing/g2_2.2.5.34_NOTASKS.bin"
#IMAGE   = "/Users/jbabcock/g2-flashing/g2_2.2.4.34.bin"
#IMAGE   = "/Users/jbabcock/g2-flashing/g2_2.2.4.34_ramloader.bin"

# channel = (service uuid, write char, notify char). Mapping by handle order:
DATA = ("00002760-08c2-11e1-9073-0e8ac72e1001",   # firmware data svc (handles 0x082x)
        "00002760-08c2-11e1-9073-0e8ac72e0001",   # write  0x0822
        "00002760-08c2-11e1-9073-0e8ac72e0002")   # notify 0x0824
CTRL = ("00002760-08c2-11e1-9073-0e8ac72e5450",   # EvenHub/heartbeat svc (handles 0x084x)
        "00002760-08c2-11e1-9073-0e8ac72e5401",   # write  0x0842
        "00002760-08c2-11e1-9073-0e8ac72e5402")   # notify 0x0844

# how far to go: 'discover' | 'heartbeat' | 'file_check' | 'flash' | 'done'
STOP_BEFORE = "done"   # FULL FLASH (begin+FILE_CHECK still gate via NAK->abort)
STAGES = ["discover","heartbeat","file_check","flash","done"]
def allowed(stage): return STAGES.index(stage) < STAGES.index(STOP_BEFORE)

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
def _nextseq():
    _seq[0]=(_seq[0]+1)&0xff; return _seq[0]
def frames(sid,pb,flag=0x00):
    body=pb+crc16(pb); tot=max(1,-(-len(body)//CHUNK)); seq=_nextseq(); out=[];off=0
    for i in range(tot):
        ch=body[off:off+CHUNK];off+=len(ch)
        out.append(bytes([0xaa,0x21,seq,len(ch),tot,i+1,sid,flag])+ch)
    return out
def ctrl_frames(op,data=b''): return frames(0xc0,bytes([op])+data)
def data_frames(block):        return frames(0xc1,block)

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

DEBUG=False
def parse_rx(frame):
    """unwrap an aa12 reply envelope -> (sid, pb). pb is [opcode,status,...]."""
    if len(frame)>=10 and frame[0]==0xaa and frame[1]==0x12:
        ln=frame[3]; sid=frame[6]; pb=frame[8:8+max(0,ln-2)]
        return sid, pb
    return None, b''
def wait_ack(br, want_op, ch_uuid, timeout=8):
    ch_uuid=ch_uuid.lower(); deadline=time.time()+timeout
    while time.time()<deadline:
        try: ch,frame=br.notes.get(timeout=max(0.1,deadline-time.time()))
        except queue.Empty: break
        sid,pb=parse_rx(frame)
        if DEBUG: print(f"    [rx ...{ch[-4:]} sid=0x{sid:02x} pb={pb.hex()}]" if sid is not None else f"    [rx ...{ch[-4:]} {frame.hex()}]")
        if ch==ch_uuid and len(pb)>=2 and pb[0]==want_op: return pb[1]
    raise TimeoutError(f"no ack op=0x{want_op:02x} on {ch_uuid}")

def send_data_msg(br, frames_list, want_op):
    svc,wch,nch=DATA
    for f in frames_list: br.write(LENS,svc,wch,f.hex(),1)
    return wait_ack(br, want_op, nch)

# ---------------- run ----------------
def run():
    br=Bridge(PHONE,TOKEN); br.start_ws(); time.sleep(2)
    print("status:",br.status())
    # fresh GATT state: stale notify/connection state makes begin time out
    try: br._req('/disconnect',{"address":LENS})
    except Exception: pass
    time.sleep(2)
    br.connect(LENS); time.sleep(2); br.discover(LENS)
    svcs=None
    for _ in range(20):
        try:
            s=br.services(LENS)
            if s and s.get('services'): svcs=s; break
        except Exception: pass
        time.sleep(1)
    print("discovery:", "ok" if svcs else "FAILED")
    if not allowed("heartbeat"):
        print(f"[stop-before={STOP_BEFORE}] discovery only; no writes."); return

    # enable notifications on the data + ctrl notify chars (give CCCD time to take)
    br.notify(LENS,DATA[0],DATA[2],True)
    br.notify(LENS,CTRL[0],CTRL[2],True)
    time.sleep(2.5)
    while not br.notes.empty(): br.notes.get()
    if not allowed("file_check"):
        print(f"[stop-before={STOP_BEFORE}] notify set up; stopping before FILE_CHECK."); return

    # ---- (gated) actual flash ----
    # heartbeat keepalive on CTRL channel during the transfer (~12s, like the app)
    hb_stop=threading.Event()
    def hb_loop():
        while not hb_stop.wait(12):
            try:
                for f in frames(0x80, bytes.fromhex("080e10266a00")): br.write(LENS,CTRL[0],CTRL[1],f.hex(),1)
            except Exception: pass
    threading.Thread(target=hb_loop,daemon=True).start()

    img=open(IMAGE,'rb').read(); N=struct.unpack_from('<I',img,8)[0]
    print(f"flashing {IMAGE} ({len(img)}B, {N} components)")
    while not br.notes.empty(): br.notes.get()          # drain stale notifications
    print("begin ack", send_data_msg(br, ctrl_frames(0x00), 0x00))
    t_start=time.time()
    for i in range(N):
        eid,off,size,crc=struct.unpack_from('<IIII',img,0x40+i*16)
        sub=img[off:off+128]; ps=struct.unpack_from('<I',sub,8)[0]; payload=img[off+128:off+128+ps]
        fn=sub[48:128].split(b'\0')[0].decode('latin1')
        print(f"[{i}] FILE_CHECK {fn} ({ps}B crc32c=0x{crc32c_msb(payload):08x})")
        st=send_data_msg(br, ctrl_frames(0x01, sub), 0x01)
        if st: hb_stop.set(); raise RuntimeError(f"FILE_CHECK NAK {st}")
        if not allowed("flash"):
            hb_stop.set(); print(f"[stop-before={STOP_BEFORE}] FILE_CHECK acked; stopping before data blocks."); return
        nb=-(-len(payload)//4096)
        for b in range(nb):
            blk=payload[b*4096:(b+1)*4096]
            for tries in range(5):
                for f in ctrl_frames(0x02): br.write(LENS,DATA[0],DATA[1],f.hex(),1)   # marker
                for f in data_frames(blk):  br.write(LENS,DATA[0],DATA[1],f.hex(),1)   # 4 KB block
                try:
                    st=wait_ack(br,0x02,DATA[2],timeout=8)
                    if st==0: break
                    print(f"   block {b} NAK={st} retry {tries}")
                except TimeoutError:
                    print(f"   block {b} ack-timeout retry {tries}")
            else:
                hb_stop.set(); raise RuntimeError(f"block {b} failed after retries")
            if b%50==0 or b==nb-1: print(f"   {fn}: block {b+1}/{nb}")
        print("   END ack", send_data_msg(br, ctrl_frames(0x03), 0x03))
    hb_stop.set()
    print(f"=== all 5 components sent in {time.time()-t_start:.0f}s; glasses should verify + reboot ===")

if __name__=="__main__":
    run()
