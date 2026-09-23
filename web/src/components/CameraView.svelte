<script lang="ts">
  // Camera preview with /keypoint_detections boxes drawn over it.
  //
  // Source choice: H.264 from /camera/preview_video when WebCodecs exists and the channel is
  // advertised, otherwise JPEG from /camera/preview. The video channel can be advertised and still
  // never send (the encoder failed to open), so no decoded frame within 2 s of subscribing drops
  // back to JPEG until the channel is advertised again.
  import { onDestroy, untrack } from "svelte";
  import { connection } from "../lib/connection";
  import { decodeCompressed, decodeImage } from "../lib/image";
  import { H264Decoder, videoDecodeSupported } from "../lib/video";

  let { showLabel = true }: { showLabel?: boolean } = $props();

  const JPEG_TOPIC = "/camera/preview";
  const VIDEO_TOPIC = "/camera/preview_video";
  const DET_TOPIC = "/keypoint_detections";
  const VIDEO_TIMEOUT_MS = 2000;
  const STALE_MS = 2000;

  interface Detection {
    x1: number;
    y1: number;
    x2: number;
    y2: number;
    conf: number;
    class_id: number;
    label: string;
    kps?: [number, number, number][];
  }
  interface Detections {
    stamp: number;
    w: number;
    h: number;
    dets: Detection[];
  }

  let canvas: HTMLCanvasElement | undefined = $state();
  let source = $state<"video" | "jpeg">("jpeg");
  let lastFrameAt = $state(0);
  let now = $state(Date.now());
  let frameSize = $state<[number, number]>([640, 360]);

  let frame: ImageBitmap | VideoFrame | null = null;
  let detections: Detections | null = null;
  let drawQueued = false;

  // The video channel id that failed, so a re-advertised channel gets another try.
  let videoFailedFor = $state<number | null>(null);
  let videoChannelId = $state<number | null>(null);

  const unsubChannels = connection.onChannels((channels) => {
    videoChannelId = channels.get(VIDEO_TOPIC)?.id ?? null;
  });
  const clock = setInterval(() => (now = Date.now()), 500);
  const unsubDets = connection.subscribeJson<Detections>(DET_TOPIC, (m) => {
    detections = m;
    queueDraw();
  });

  $effect(() => {
    const id = videoChannelId;
    const useVideo = videoDecodeSupported() && id !== null && id !== videoFailedFor;
    return untrack(() => (useVideo ? startVideo(id) : startJpeg()));
  });

  function setFrame(next: ImageBitmap | VideoFrame) {
    frame?.close();
    frame = next;
    const w = "displayWidth" in next ? next.displayWidth : next.width;
    const h = "displayHeight" in next ? next.displayHeight : next.height;
    if (w !== frameSize[0] || h !== frameSize[1]) frameSize = [w, h];
    lastFrameAt = Date.now();
    queueDraw();
  }

  function startJpeg(): () => void {
    source = "jpeg";
    let decoding = false;
    let closed = false;
    const unsub = connection.subscribeRaw(JPEG_TOPIC, (m) => {
      // One decode in flight at a time; frames that arrive meanwhile are skipped.
      if (decoding) return;
      decoding = true;
      decodeImage(m.channel, m.data)
        .then((bitmap) => (closed ? bitmap.close() : setFrame(bitmap)))
        .catch((err) => console.warn("preview decode failed", err))
        .finally(() => (decoding = false));
    });
    return () => {
      closed = true;
      unsub();
    };
  }

  function startVideo(channelId: number): () => void {
    source = "video";
    let closed = false;
    let decoded = false;
    const giveUp = (why: string) => {
      if (closed) return;
      console.warn(`H.264 preview unavailable (${why}); using JPEG`);
      videoFailedFor = channelId;
    };
    const decoder = new H264Decoder(
      (f) => {
        if (closed) return f.close();
        decoded = true;
        setFrame(f);
      },
      (err) => giveUp(err.message),
    );
    const unsub = connection.subscribeRaw(VIDEO_TOPIC, (m) => {
      try {
        decoder.push(decodeCompressed(m.channel, m.data).data);
      } catch (err) {
        giveUp(err instanceof Error ? err.message : String(err));
      }
    });
    const timer = setTimeout(() => {
      if (!decoded) giveUp(`no frame within ${VIDEO_TIMEOUT_MS} ms`);
    }, VIDEO_TIMEOUT_MS);
    return () => {
      closed = true;
      clearTimeout(timer);
      unsub();
      decoder.close();
    };
  }

  function queueDraw() {
    if (drawQueued) return;
    drawQueued = true;
    requestAnimationFrame(() => {
      drawQueued = false;
      draw();
    });
  }

  function draw() {
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    const [w, h] = frameSize;
    if (canvas.width !== w || canvas.height !== h) {
      canvas.width = w;
      canvas.height = h;
    }
    ctx.fillStyle = "#141518";
    ctx.fillRect(0, 0, w, h);
    if (frame) ctx.drawImage(frame, 0, 0, w, h);
    if (!detections || detections.w <= 0 || detections.h <= 0) return;

    const sx = w / detections.w;
    const sy = h / detections.h;
    const px = Math.max(1, w / 320);
    ctx.lineWidth = 2 * px;
    ctx.font = `${Math.round(12 * px)}px "IBM Plex Mono", monospace`;
    ctx.textBaseline = "bottom";
    for (const d of detections.dets) {
      const color = d.label === "opponent" || d.label === "house_bot" ? "#FF6B4A" : "#F2F1EC";
      const x = d.x1 * sx;
      const y = d.y1 * sy;
      const bw = (d.x2 - d.x1) * sx;
      const bh = (d.y2 - d.y1) * sy;
      ctx.strokeStyle = color;
      ctx.strokeRect(x, y, bw, bh);

      const text = `${d.label} ${d.conf.toFixed(2)}`;
      const tw = ctx.measureText(text).width + 8 * px;
      const th = 16 * px;
      const ty = y - th < 0 ? y + bh : y - th;
      ctx.fillStyle = color;
      ctx.fillRect(x, ty, tw, th);
      ctx.fillStyle = "#121212";
      ctx.fillText(text, x + 4 * px, ty + th - 2 * px);

      d.kps?.forEach(([kx, ky, kc], i) => {
        if (kc < 0.3) return;
        ctx.beginPath();
        ctx.arc(kx * sx, ky * sy, 4 * px, 0, Math.PI * 2);
        if (i === 0) {
          ctx.fillStyle = color;
          ctx.fill();
        } else {
          ctx.strokeStyle = color;
          ctx.lineWidth = 1.5 * px;
          ctx.stroke();
          ctx.lineWidth = 2 * px;
        }
      });
    }
  }

  $effect(() => {
    if (canvas) queueDraw();
  });

  onDestroy(() => {
    unsubChannels();
    unsubDets();
    clearInterval(clock);
    frame?.close();
    frame = null;
  });

  const stale = $derived(now - lastFrameAt > STALE_MS);
</script>

<div class="camera">
  <canvas bind:this={canvas} aria-label="Camera preview with detections"></canvas>
  {#if showLabel}<span class="tag">CAMERA</span>{/if}
  {#if stale}
    <span class="nosignal mono">NO PREVIEW</span>
  {/if}
  <span class="src mono">{source === "video" ? "H.264" : "JPEG"}</span>
</div>

<style>
  .camera {
    position: relative;
    width: 100%;
    aspect-ratio: 16 / 9;
    border: 2px solid var(--ink);
    background: var(--video-bg);
    overflow: hidden;
  }
  canvas {
    display: block;
    width: 100%;
    height: 100%;
    object-fit: contain;
  }
  .tag {
    position: absolute;
    left: 0;
    top: 0;
    padding: 5px 9px;
    background: var(--ink);
    color: var(--bg);
    font-weight: 700;
    font-size: 11px;
    letter-spacing: 0.1em;
  }
  .nosignal {
    position: absolute;
    inset: 0;
    display: flex;
    align-items: center;
    justify-content: center;
    color: #8a8a85;
    font-size: 13px;
    letter-spacing: 0.1em;
  }
  .src {
    position: absolute;
    right: 6px;
    bottom: 4px;
    color: #8a8a85;
    font-size: 10px;
  }
</style>
