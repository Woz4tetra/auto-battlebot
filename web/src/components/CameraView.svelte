<script lang="ts">
  // Camera preview with the field outline (/status/tracks) and /keypoint_detections boxes drawn
  // over it. Each robot keeps one color (lib/robots.ts), shared with the top-down view.
  //
  // Source choice: H.264 from /camera/preview_video when WebCodecs exists and the channel is
  // advertised, otherwise JPEG from /camera/preview. The video channel can be advertised and still
  // never send (the encoder failed to open), so no decoded frame within 2 s of subscribing drops
  // back to JPEG until the channel is advertised again.
  import { onDestroy, untrack } from "svelte";
  import { connection } from "../lib/connection";
  import { decodeCompressed, decodeImage } from "../lib/image";
  import { H264Decoder, videoDecodeSupported } from "../lib/video";
  import { displayLabel, robotColor } from "../lib/robots";
  import { status } from "../lib/status.svelte";

  let { showLabel = true }: { showLabel?: boolean } = $props();

  const JPEG_TOPIC = "/camera/preview";
  const VIDEO_TOPIC = "/camera/preview_video";
  const DET_TOPIC = "/keypoint_detections";
  const VIDEO_TIMEOUT_MS = 2000;
  const STALE_MS = 2000;
  const OUTLINE_COLOR = "rgba(61, 214, 140, 0.9)";

  interface Rect {
    x: number;
    y: number;
    w: number;
    h: number;
  }
  const overlapArea = (a: Rect, b: Rect) =>
    Math.max(0, Math.min(a.x + a.w, b.x + b.w) - Math.max(a.x, b.x)) *
    Math.max(0, Math.min(a.y + a.h, b.y + b.h) - Math.max(a.y, b.y));

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
    const px = Math.max(1, w / 320);
    drawOutline(ctx, w, h, px);
    if (!detections || detections.w <= 0 || detections.h <= 0) return;

    const sx = w / detections.w;
    const sy = h / detections.h;
    const ours = new Set(status.tracks?.robots.filter((r) => r.ours).map((r) => r.label) ?? []);
    const boxes = detections.dets.map((d) => ({
      d,
      color: robotColor(d.label, ours.has(d.label)),
      rect: { x: d.x1 * sx, y: d.y1 * sy, w: (d.x2 - d.x1) * sx, h: (d.y2 - d.y1) * sy },
    }));

    // Boxes and keypoints first, then every label, so no box stroke runs over a label.
    for (const { d, color, rect } of boxes) {
      ctx.strokeStyle = color;
      ctx.lineWidth = 2 * px;
      ctx.strokeRect(rect.x, rect.y, rect.w, rect.h);
      d.kps?.forEach(([kx, ky, kc], i) => {
        if (kc < 0.3) return;
        ctx.beginPath();
        ctx.arc(kx * sx, ky * sy, 3.5 * px, 0, Math.PI * 2);
        if (i === 0) {
          ctx.fillStyle = color;
          ctx.fill();
        } else {
          ctx.lineWidth = 1.5 * px;
          ctx.stroke();
        }
      });
    }
    drawLabels(ctx, boxes, w, h, px);
  }

  function drawOutline(ctx: CanvasRenderingContext2D, w: number, h: number, px: number) {
    const outline = status.appUp ? status.tracks?.field_outline : undefined;
    if (!outline?.length) return;
    ctx.save();
    ctx.strokeStyle = OUTLINE_COLOR;
    ctx.lineWidth = 2 * px;
    ctx.setLineDash([6 * px, 4 * px]);
    ctx.lineJoin = "round";
    for (const line of outline) {
      ctx.beginPath();
      line.forEach((p, i) => (i ? ctx.lineTo(p.u * w, p.v * h) : ctx.moveTo(p.u * w, p.v * h)));
      ctx.stroke();
    }
    ctx.restore();
  }

  // Labels sit outside their box: above, else below, else inside the top edge. Among those, the
  // spot that covers the least of the other boxes and the labels already placed wins, so labels
  // on overlapping boxes step aside instead of hiding the box behind them.
  function drawLabels(
    ctx: CanvasRenderingContext2D,
    boxes: { d: Detection; color: string; rect: Rect }[],
    w: number,
    h: number,
    px: number,
  ) {
    ctx.font = `600 ${Math.round(10 * px)}px "IBM Plex Mono", monospace`;
    ctx.textBaseline = "middle";
    const th = 14 * px;
    const placed: Rect[] = [];
    const order = [...boxes].sort((a, b) => b.d.conf - a.d.conf);
    for (const { d, color, rect } of order) {
      const text = `${displayLabel(d.label)} ${Math.round(d.conf * 100)}%`;
      const tw = ctx.measureText(text).width + 10 * px;
      const clampX = (x: number) => Math.min(Math.max(0, x), Math.max(0, w - tw));
      const candidates: Rect[] = [
        { x: clampX(rect.x), y: rect.y - th - 2 * px, w: tw, h: th },
        { x: clampX(rect.x + rect.w - tw), y: rect.y - th - 2 * px, w: tw, h: th },
        { x: clampX(rect.x), y: rect.y + rect.h + 2 * px, w: tw, h: th },
        { x: clampX(rect.x + rect.w - tw), y: rect.y + rect.h + 2 * px, w: tw, h: th },
        { x: clampX(rect.x), y: rect.y + 2 * px, w: tw, h: th },
      ].filter((c) => c.y >= 0 && c.y + c.h <= h);
      if (!candidates.length) candidates.push({ x: clampX(rect.x), y: 0, w: tw, h: th });
      const cost = (c: Rect) =>
        placed.reduce((sum, p) => sum + overlapArea(c, p) * 4, 0) +
        boxes.reduce((sum, b) => (b.rect === rect ? sum : sum + overlapArea(c, b.rect)), 0);
      // Ties keep the earlier candidate, so an unobstructed label always sits above-left.
      let best = candidates[0];
      let bestCost = cost(best);
      for (const c of candidates.slice(1)) {
        const cc = cost(c);
        if (cc < bestCost) {
          best = c;
          bestCost = cc;
        }
      }
      placed.push(best);
      // Dark translucent plate with a color bar: the frame behind stays readable.
      ctx.fillStyle = "rgba(10, 10, 10, 0.72)";
      ctx.fillRect(best.x, best.y, best.w, best.h);
      ctx.fillStyle = color;
      ctx.fillRect(best.x, best.y, 3 * px, best.h);
      ctx.fillText(text, best.x + 6 * px, best.y + best.h / 2 + 0.5 * px);
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
