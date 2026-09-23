// H.264 preview decode with WebCodecs.
//
// Each /camera/preview_video message is one Annex-B access unit. Keyframes carry SPS and PPS in
// band, so the decoder needs no `description`: it is configured from the SPS of the first
// keyframe, and reconfigured if a later keyframe carries a different SPS.

export function videoDecodeSupported(): boolean {
  return typeof VideoDecoder !== "undefined" && typeof EncodedVideoChunk !== "undefined";
}

const NAL_IDR = 5;
const NAL_SPS = 7;

interface AccessUnit {
  keyframe: boolean;
  sps: Uint8Array | null;
}

/** Walks Annex-B start codes (00 00 01 or 00 00 00 01) and reads the NAL unit types. */
export function scanAccessUnit(data: Uint8Array): AccessUnit {
  let keyframe = false;
  let sps: Uint8Array | null = null;
  let i = 0;
  const n = data.length;
  const nalStarts: number[] = [];
  while (i + 3 <= n) {
    if (data[i] === 0 && data[i + 1] === 0 && data[i + 2] === 1) {
      nalStarts.push(i + 3);
      i += 3;
    } else {
      i += 1;
    }
  }
  for (let k = 0; k < nalStarts.length; k++) {
    const start = nalStarts[k];
    if (start >= n) continue;
    const type = data[start] & 0x1f;
    if (type === NAL_IDR) keyframe = true;
    if (type === NAL_SPS && !sps) {
      // The NAL ends at the next start code; trailing zero bytes belong to that start code.
      let end = k + 1 < nalStarts.length ? nalStarts[k + 1] - 3 : n;
      while (end > start && data[end - 1] === 0) end--;
      sps = data.subarray(start, end);
    }
  }
  return { keyframe, sps };
}

/** avc1.PPCCLL from the SPS profile_idc, constraint flags, and level_idc bytes. */
export function codecFromSps(sps: Uint8Array): string {
  const hex = (b: number) => b.toString(16).padStart(2, "0");
  return `avc1.${hex(sps[1])}${hex(sps[2])}${hex(sps[3])}`;
}

function sameBytes(a: Uint8Array | null, b: Uint8Array): boolean {
  if (!a || a.length !== b.length) return false;
  for (let i = 0; i < a.length; i++) if (a[i] !== b[i]) return false;
  return true;
}

/** More queued chunks than this means decode is falling behind; drop to the next keyframe. */
const MAX_QUEUE = 8;

export class H264Decoder {
  #decoder: VideoDecoder | null = null;
  #sps: Uint8Array | null = null;
  #waitForKeyframe = true;
  #timestampUs = 0;
  #onFrame: (frame: VideoFrame) => void;
  #onError: (err: Error) => void;

  constructor(onFrame: (frame: VideoFrame) => void, onError: (err: Error) => void) {
    this.#onFrame = onFrame;
    this.#onError = onError;
  }

  push(data: Uint8Array): void {
    const unit = scanAccessUnit(data);
    if (unit.keyframe && unit.sps && !sameBytes(this.#sps, unit.sps)) {
      if (!this.#configure(unit.sps)) return;
    }
    const decoder = this.#decoder;
    if (!decoder || decoder.state !== "configured") return;
    if (this.#waitForKeyframe && !unit.keyframe) return;
    if (decoder.decodeQueueSize > MAX_QUEUE && !unit.keyframe) {
      this.#waitForKeyframe = true;
      return;
    }
    this.#waitForKeyframe = false;
    // Timestamps only need to increase; 33 ms steps keep them readable in a debugger.
    this.#timestampUs += 33_333;
    try {
      decoder.decode(
        new EncodedVideoChunk({
          type: unit.keyframe ? "key" : "delta",
          timestamp: this.#timestampUs,
          data: data.slice(),
        }),
      );
    } catch (err) {
      this.#fail(err);
    }
  }

  close(): void {
    const decoder = this.#decoder;
    this.#decoder = null;
    this.#sps = null;
    if (decoder && decoder.state !== "closed") decoder.close();
  }

  #configure(sps: Uint8Array): boolean {
    this.close();
    try {
      const decoder = new VideoDecoder({
        output: (frame) => this.#onFrame(frame),
        error: (err) => this.#fail(err),
      });
      decoder.configure({ codec: codecFromSps(sps), optimizeForLatency: true });
      this.#decoder = decoder;
      this.#sps = sps.slice();
      this.#waitForKeyframe = true;
      return true;
    } catch (err) {
      this.#fail(err);
      return false;
    }
  }

  #fail(err: unknown): void {
    this.close();
    this.#onError(err instanceof Error ? err : new Error(String(err)));
  }
}
