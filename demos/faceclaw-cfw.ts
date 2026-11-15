// Faceclaw CFW revision detection.
//
// The firmware appends protobuf field 100 to the sid=0x09 settings READ
// response: "Faceclaw/<n>". There are no feature tokens; scripts gate on the
// revision number. g2-kit's queryCapabilities only accepts the older EVENCFW
// prefix, so this does the same settings read and parses field 100 itself.

import { create, toBinary } from "@bufbuild/protobuf";
import type { G2SessionLike } from "g2-kit/ble";
import { SID_UI_SETTING } from "g2-kit/ble";
import {
  APPRequestSettingType,
  G2SettingPackageSchema,
  g2_settingCommandId,
} from "./node_modules/g2-kit/ble/gen/g2_setting_pb";

export const FACECLAW_CFW_MAGIC = "Faceclaw/";

export interface FaceclawCfw {
  /** Firmware revision after the slash (39 for "Faceclaw/39"). */
  revision: number;
  /** The full advertised string, verbatim. */
  raw: string;
}

/** First top-level length-delimited field `fieldNo` of a protobuf message. */
export function findLenDelimField(buf: Uint8Array, fieldNo: number): Uint8Array | null {
  let p = 0;
  const readVarint = (): number => {
    let v = 0, shift = 0;
    while (p < buf.length) {
      const b = buf[p++]!;
      v |= (b & 0x7f) << shift;
      if ((b & 0x80) === 0) break;
      shift += 7;
    }
    return v >>> 0;
  };
  while (p < buf.length) {
    const tag = readVarint();
    const field = tag >>> 3;
    const wire = tag & 7;
    if (wire === 0) readVarint();
    else if (wire === 2) {
      const len = readVarint();
      if (field === fieldNo) return buf.subarray(p, p + len);
      p += len;
    } else if (wire === 5) p += 4;
    else if (wire === 1) p += 8;
    else return null;
  }
  return null;
}

export function parseFaceclawCfw(settingsPb: Uint8Array): FaceclawCfw | null {
  const bytes = findLenDelimField(settingsPb, 100);
  if (!bytes) return null;
  const raw = new TextDecoder("utf-8", { fatal: false }).decode(bytes).replace(/\0+$/, "");
  if (!raw.startsWith(FACECLAW_CFW_MAGIC)) return null;
  const revision = Number(raw.slice(FACECLAW_CFW_MAGIC.length).split(/\s+/)[0]);
  if (!Number.isInteger(revision) || revision <= 0) return null;
  return { revision, raw };
}

/** Settings read plus field-100 parse. Null on stock firmware or ack timeout. */
export async function queryFaceclawCfw(session: G2SessionLike, magic: number): Promise<FaceclawCfw | null> {
  const req = create(G2SettingPackageSchema, {
    commandId: g2_settingCommandId.DeviceReceiveRequest,
    magicRandom: magic,
    deviceReceiveRequestFromApp: {
      settingInfoType: APPRequestSettingType.APP_REQUIRE_BASIC_SETTING,
    },
  });
  const pb = toBinary(G2SettingPackageSchema, req);
  const ack = await session.sendPb(SID_UI_SETTING, pb, magic, { ackTimeoutMs: 4000 });
  if (!ack) return null;
  return parseFaceclawCfw(ack.pb);
}
