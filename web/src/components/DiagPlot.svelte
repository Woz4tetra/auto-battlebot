<script lang="ts">
  // Last 60 s of one numeric diagnostics value.
  import { diagnostics } from "../lib/diagnostics.svelte";

  let {
    module,
    valueKey,
    height = 132,
  }: { module: string; valueKey: string; height?: number } = $props();

  const W = 740;
  const H = 128;
  const WINDOW_MS = 60_000;

  const plot = $derived.by(() => {
    void diagnostics.version;
    const series = diagnostics.history(module, valueKey);
    if (!series || series.v.length === 0) return null;
    let min = Math.min(...series.v);
    let max = Math.max(...series.v);
    if (min === max) {
      const pad = Math.abs(min) * 0.1 || 1;
      min -= pad;
      max += pad;
    }
    const end = series.t[series.t.length - 1];
    const y = (v: number) => H - 8 - ((v - min) / (max - min)) * (H - 16);
    const x = (t: number) => W - ((end - t) / WINDOW_MS) * W;
    const points = series.t.map((t, i) => `${x(t).toFixed(1)},${y(series.v[i]).toFixed(1)}`);
    return { points: points.join(" "), min, max };
  });

  const fmt = (v: number) => (Math.abs(v) >= 100 ? v.toFixed(0) : v.toPrecision(3));
</script>

<div class="plot">
  <span class="head label">
    <span>{valueKey.toUpperCase()} · LAST 60 S</span>
    <span class="range mono">{plot ? `${fmt(plot.min)} to ${fmt(plot.max)}` : "no data"}</span>
  </span>
  <div class="frame" style:height="{height}px">
    <svg viewBox="0 0 {W} {H}" preserveAspectRatio="none" role="img" aria-label="Recent values">
      <line x1="0" y1="32" x2={W} y2="32" />
      <line x1="0" y1="64" x2={W} y2="64" />
      <line x1="0" y1="96" x2={W} y2="96" />
      {#if plot}
        <polyline points={plot.points} />
      {/if}
    </svg>
  </div>
</div>

<style>
  .plot {
    display: flex;
    flex-direction: column;
    gap: 8px;
  }
  .head {
    display: flex;
    justify-content: space-between;
    gap: 12px;
    min-width: 0;
  }
  .head > span:first-child {
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .range {
    letter-spacing: 0;
    color: var(--muted);
    white-space: nowrap;
  }
  .frame {
    border: 2px solid var(--ink);
  }
  svg {
    display: block;
    width: 100%;
    height: 100%;
  }
  line {
    stroke: var(--grid);
    stroke-width: 1;
    vector-effect: non-scaling-stroke;
  }
  polyline {
    fill: none;
    stroke: var(--ink);
    stroke-width: 2;
    stroke-linejoin: round;
    vector-effect: non-scaling-stroke;
  }
</style>
