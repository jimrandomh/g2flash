import { describe, expect, test } from "bun:test";
import { HidKeys, parseReportMap, reportKeys } from "./keyboard-hid";
import { KeyboardRecords, keyboardControl, le16, u16 } from "./keyboard-protocol";
import { KeyboardClient, attributeUuid } from "./keyboard-client";
import { findLenDelimField } from "./glassly-cfw";

const boot = Uint8Array.from([
  0x05,1,0x09,6,0xa1,1,0x05,7,0x19,0xe0,0x29,0xe7,0x15,0,0x25,1,0x75,1,0x95,8,0x81,2,
  0x75,8,0x95,1,0x81,1,0x95,6,0x75,8,0x15,0,0x25,0x65,0x05,7,0x19,0,0x29,0x65,0x81,0,0xc0,
]);
const numbered = Uint8Array.from([
  ...boot.slice(0, 6), 0x85,1,...boot.slice(6, -1),
  0x85,2,0x05,0x0c,0x15,0,0x25,1,0x75,1,0x95,3,0x09,0xe9,0x09,0xea,0x09,0xe2,0x81,2,
  0x75,5,0x95,1,0x81,1,0xc0,
]);
function frame(kind: number, seq: number, epoch: number, request: number, code: number, handle: number, data: Uint8Array, offset = 0, total = data.length): Uint8Array {
  const p = new Uint8Array(24 + data.length), v = new DataView(p.buffer);
  p.set([75,66,1,kind]);v.setUint32(4,seq,true);v.setUint16(8,epoch,true);v.setUint16(10,request,true);
  v.setUint16(12,code,true);v.setUint16(14,handle,true);v.setUint16(16,total,true);v.setUint16(18,offset,true);
  v.setUint32(20,1234,true);p.set(data,24);return p;
}
describe("descriptor-driven HID", () => {
  test("boot array, modifiers, release, repeats and rollover", () => {
    const r = parseReportMap(boot).get(0)!, keys = new HidKeys();
    expect(keys.update(12,r,Uint8Array.of(2,0,4,0,0,0,0,0))).toEqual([
      {page:7,usage:0xe1,down:true},{page:7,usage:4,down:true},
    ]);
    expect(keys.update(12,r,Uint8Array.of(2,0,4,0,0,0,0,0))).toEqual([]);
    expect(() => keys.update(12,r,Uint8Array.of(0,0,1,1,1,1,1,1))).toThrow("rollover");
    expect(keys.reset().map(k => k.down)).toEqual([false,false]);
    expect(() => reportKeys(r,Uint8Array.of(0))).toThrow("length");
  });
  test("HOGP report IDs are out of band, with consumer keys", () => {
    const maps = parseReportMap(numbered);
    expect(reportKeys(maps.get(1)!,Uint8Array.of(0,0,5,0,0,0,0,0))).toEqual([{page:7,usage:5}]);
    expect(reportKeys(maps.get(2)!,Uint8Array.of(5))).toEqual([{page:12,usage:0xe9},{page:12,usage:0xe2}]);
    expect(() => reportKeys(maps.get(2)!,Uint8Array.of(2,5))).toThrow("length");
  });
  test("NKRO bitmap with non-byte-aligned fields and report union", () => {
    const map = Uint8Array.of(5,7,0x19,4,0x29,19,0x15,0,0x25,1,0x75,1,0x95,16,0x81,2);
    const r = parseReportMap(map).get(0)!, keys = new HidKeys();
    expect(reportKeys(r,Uint8Array.of(0x81,0x80))).toEqual([{page:7,usage:4},{page:7,usage:11},{page:7,usage:19}]);
    expect(keys.update(10,r,Uint8Array.of(1,0)).length).toBe(1);
    expect(keys.update(11,r,Uint8Array.of(1,0))).toEqual([]);
    expect(keys.update(10,r,Uint8Array.of(0,0))).toEqual([]);
    expect(keys.update(11,r,Uint8Array.of(0,0))).toEqual([{page:7,usage:4,down:false}]);
  });
  test("array values index usages relative to logical minimum", () => {
    const r = parseReportMap(Uint8Array.of(5,7,0x19,4,0x29,6,0x15,1,0x25,3,0x75,8,0x95,1,0x81,0)).get(0)!;
    expect(reportKeys(r,Uint8Array.of(2))).toEqual([{page:7,usage:5}]);
    expect(reportKeys(r,Uint8Array.of(0))).toEqual([]);
  });
  test("descriptor resource bounds and malformed items", () => {
    for (const data of [new Uint8Array(),new Uint8Array(513),Uint8Array.of(0xfe),Uint8Array.of(0x75),
      Uint8Array.of(0xb4),Uint8Array.of(0xc0),Uint8Array.of(0x85,0),Uint8Array.of(0x75,33,0x95,1,0x81,2),
      Uint8Array.of(0x75,32,0x96,0,2,0x81,2),Uint8Array.of(0xa1,1)]) expect(() => parseReportMap(data)).toThrow();
    // Deterministic malformed corpus; parsing must be bounded and never hang.
    let seed=1;
    for(let i=0;i<1000;i++) {
      const data = Uint8Array.from({length:i%128+1},()=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed>>>24;});
      try { parseReportMap(data); } catch (e) { expect(e).toBeInstanceOf(Error); }
    }
  });
});
describe("keyboard relay protocol", () => {
  test("16-bit UUIDs encoded using the Bluetooth 128-bit base", () => {
    expect(attributeUuid(Uint8Array.of(0xfb,0x34,0x9b,0x5f,0x80,0,0,0x80,0,0x10,0,0,0x12,0x18,0,0))).toBe(0x1812);
  });
  test("wire control has the dedicated field and epoch", () => {
    const field = findLenDelimField(keyboardControl(4,0x1234,0x5678,[1,2,3,4,5,6,7]),130)!;
    expect(Array.from(field)).toEqual([75,66,1,4,0x34,0x12,0x78,0x56,1,2,3,4,5,6,7]);
  });
  test("fragments, duplicate delivery, gaps, epochs and sequence wrap", () => {
    let gaps=0;const stream=new KeyboardRecords(()=>gaps++);
    const data=Uint8Array.from({length:200},(_,i)=>i);
    expect(stream.push(frame(3,0xffffffff,1,0,13,33,data.slice(0,96),0,200))).toBeUndefined();
    expect(stream.push(frame(3,0xffffffff,1,0,13,33,data.slice(96,192),96,200))).toBeUndefined();
    expect(stream.push(frame(3,0xffffffff,1,0,13,33,data.slice(192),192,200))!.data).toEqual(data);
    const status=frame(1,0,1,0,0,0,new Uint8Array(10));
    expect(stream.push(status)!.sequence).toBe(0);
    expect(stream.push(status)).toBeUndefined();expect(gaps).toBe(1);
    stream.push(frame(1,2,1,0,0,0,new Uint8Array(10)));expect(gaps).toBe(2);
    expect(stream.push(status)).toBeUndefined();expect(gaps).toBe(2);
    stream.push(frame(1,3,2,0,0,0,new Uint8Array(10)));expect(gaps).toBe(3);
  });
  test("missing/mismatched fragments never emit partial input", () => {
    let gaps=0;const stream=new KeyboardRecords(()=>gaps++);
    stream.push(frame(3,1,1,0,13,10,new Uint8Array(96),0,100));
    expect(stream.push(frame(3,1,1,0,13,11,new Uint8Array(4),96,100))).toBeUndefined();
    expect(gaps).toBe(2);
    expect(stream.push(frame(3,1,1,0,13,10,new Uint8Array(4),96,100))).toBeUndefined();
    expect(stream.push(new Uint8Array(513))).toBeUndefined();
  });
});

test("full simulated HOGP discovery, long Report Map, subscriptions, typing and loss recovery", async () => {
  const attrs = new Map<number,{uuid:number,value:Uint8Array}>();
  const add=(h:number,uuid:number,bytes:number[]|Uint8Array)=>attrs.set(h,{uuid,value:Uint8Array.from(bytes)});
  const mockMap=Uint8Array.from([...numbered.slice(0,-1),0x85,3,0x05,6,0x09,0x20,0x15,0,0x25,100,0x75,8,0x95,1,0x81,2,0xc0]);
  add(1,0x2800,[1,0x18]);add(2,0x2803,[0x20,3,0,5,0x2a]);add(3,0x2a05,[]);add(4,0x2902,[0,0]);
  add(20,0x2800,[0x12,0x18]);
  add(21,0x2803,[2,22,0,0x4b,0x2a]);add(22,0x2a4b,mockMap);add(23,0x2907,[0x19,0x2a]);
  add(24,0x2803,[6,25,0,0x4e,0x2a]);add(25,0x2a4e,[1]);
  add(26,0x2803,[0x12,27,0,0x4d,0x2a]);add(27,0x2a4d,[]);add(28,0x2902,[0,0]);add(29,0x2908,[1,1]);
  add(30,0x2803,[0x12,31,0,0x4d,0x2a]);add(31,0x2a4d,[]);add(32,0x2902,[0,0]);add(33,0x2908,[2,1]);
  add(40,0x2800,[0x0f,0x18]);add(41,0x2803,[0x12,42,0,0x19,0x2a]);add(42,0x2a19,[100]);add(43,0x2902,[0,0]);add(44,0x2908,[3,1]);
  let seq=0,epoch=0;const writes:number[]=[];let watches:number[]=[];
  const events:{down:boolean;usage:number}[]=[];
  let client:KeyboardClient;
  const emit=(kind:number,request:number,code:number,handle:number,data:Uint8Array)=>client.accept(frame(kind,++seq,epoch,request,code,handle,data));
  client=new KeyboardClient({async send(pb) {
    const p=findLenDelimField(pb,130)!,op=p[3]!,req=u16(p,4),h=u16(p,8);
    if(op===1)emit(1,req,0,0,Uint8Array.of(1,0,0,0,0,0,0,0,0,0));
    else if(op===4){epoch++;emit(1,req,0,0,Uint8Array.of(1,3,1,1,0,0,0,0,0,0));}
    else if(op===9) {
      const batch=[...attrs].filter(([handle])=>handle>=h).slice(0,4);
      emit(2,req,batch.length?2:(10<<8)|2,h,Uint8Array.from(batch.length?[1,...batch.flatMap(([handle,a])=>[...le16(handle),...le16(a.uuid)])]:[]));
    } else if(op===10) {
      const data=attrs.get(h)!.value,offset=u16(p,10);
      emit(2,req,offset>=data.length?(7<<8)|6:offset?6:5,h,data.slice(offset,offset+20));
    } else if(op===11) {writes.push(h);emit(2,req,p[10]?10:9,h,new Uint8Array());}
    else if(op===12){watches=Array.from({length:p[8]!},(_,i)=>u16(p,9+2*i));emit(1,req,0,0,Uint8Array.of(1,3,1,1,0,0,0,0,0,0));}
    else throw new Error(`Unexpected op ${op}`);
  }},{keys(e){events.push(...e);}});
  await client.enable();await client.connect(1,new Uint8Array(6));const config=await client.configure();
  expect(watches).toEqual([27,31,42,3]);expect(writes).toEqual([25,4,28,32,43]);expect(config.reports.map(r=>r.id)).toEqual([1,2,3]);
  emit(3,0,13,27,Uint8Array.of(0,0,4,0,0,0,0,0));emit(3,0,13,31,Uint8Array.of(1));
  expect(events.map(e=>[e.usage,e.down])).toEqual([[4,true],[0xe9,true]]);
  seq++; // release lost in firmware queue or phone notification path
  emit(1,0,0,0,Uint8Array.of(1,3,1,1,0,0,0,0,0,0));
  expect(events.slice(-2).map(e=>e.down)).toEqual([false,false]);
  const restored = new KeyboardClient({async send(){}},{keys(e){events.push(...e);}});
  restored.restore(config);restored.accept(frame(3,++seq,epoch,0,13,27,Uint8Array.of(0,0,5,0,0,0,0,0)));
  expect(events.at(-1)).toMatchObject({usage:5,down:true});restored.lostPhoneLink();
  expect(events.at(-1)).toMatchObject({usage:5,down:false});
  emit(3,0,14,3,Uint8Array.of(1,0,255,255));
  const count=events.length;emit(3,0,13,27,Uint8Array.of(0,0,6,0,0,0,0,0));
  expect(events.length).toBe(count); // invalidated handles cannot generate keys
  client.close();restored.close();
});
