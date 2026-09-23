// Protobuf decode for foxglove.CompressedImage and foxglove.CompressedVideo.
//
// The relay advertises protobuf channels with a base64 FileDescriptorSet as the schema. The type
// is built from that at runtime, the same way Foxglove Studio does it, so the page never ships
// its own copy of the .proto files.

import protobuf from "protobufjs";
import "protobufjs/ext/descriptor";
import type { Channel } from "@foxglove/ws-protocol";

export interface CompressedMessage {
  /** Capture time in seconds. */
  stamp: number;
  frame_id: string;
  data: Uint8Array;
  format: string;
}

const typeCache = new Map<string, protobuf.Type>();

function base64ToBytes(b64: string): Uint8Array {
  const bin = atob(b64);
  const out = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
  return out;
}

/** Looks up (and caches) the message type for a protobuf channel. */
export function protobufType(channel: Channel): protobuf.Type {
  const key = `${channel.schemaName}\n${channel.schema}`;
  let type = typeCache.get(key);
  if (!type) {
    if (channel.schemaEncoding !== "protobuf") {
      throw new Error(
        `${channel.topic}: expected a protobuf schema, got ${channel.schemaEncoding}`,
      );
    }
    const root = protobuf.Root.fromDescriptor(base64ToBytes(channel.schema), { keepCase: true });
    type = root.lookupType(channel.schemaName);
    typeCache.set(key, type);
  }
  return type;
}

interface RawCompressed {
  timestamp?: { seconds?: number | { toNumber(): number }; nanos?: number };
  frame_id?: string;
  data?: Uint8Array;
  format?: string;
}

function toNumber(v: number | { toNumber(): number } | undefined): number {
  if (v === undefined) return 0;
  return typeof v === "number" ? v : v.toNumber();
}

/** Decodes a CompressedImage or CompressedVideo; both share these four fields. */
export function decodeCompressed(channel: Channel, bytes: Uint8Array): CompressedMessage {
  const msg = protobufType(channel).decode(bytes) as unknown as RawCompressed;
  const ts = msg.timestamp;
  return {
    stamp: toNumber(ts?.seconds) + (ts?.nanos ?? 0) * 1e-9,
    frame_id: msg.frame_id ?? "",
    data: msg.data ?? new Uint8Array(),
    format: msg.format ?? "",
  };
}

/** CompressedImage payload -> ImageBitmap. */
export async function decodeImage(channel: Channel, bytes: Uint8Array): Promise<ImageBitmap> {
  const msg = decodeCompressed(channel, bytes);
  const format = msg.format.toLowerCase();
  const mime = format.includes("png") ? "image/png" : "image/jpeg";
  // Copy into a fresh ArrayBuffer so the Blob never aliases the socket's receive buffer.
  const blob = new Blob([msg.data.slice()], { type: mime });
  return createImageBitmap(blob);
}
