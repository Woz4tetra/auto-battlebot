<script lang="ts">
  // /status/tracks on the field outline. Field frame: origin at the field center, meters.
  // Drawn with +x to the right and +y up (away from the camera marker at the bottom).
  import { displayLabel, trackColor } from "../lib/robots";
  import { status } from "../lib/status.svelte";

  let { showLabel = true }: { showLabel?: boolean } = $props();

  const VIEW = 240;
  const MARGIN = 24;
  const BOX = VIEW - 2 * MARGIN;

  const tracks = $derived(status.appUp ? status.tracks : null);
  const hasField = $derived(!!tracks && tracks.field_x > 0 && tracks.field_y > 0);
  const scale = $derived(hasField ? BOX / Math.max(tracks!.field_x, tracks!.field_y) : 1);
  const fieldW = $derived(hasField ? tracks!.field_x * scale : BOX);
  const fieldH = $derived(hasField ? tracks!.field_y * scale : BOX);
  const left = $derived((VIEW - fieldW) / 2);
  const top = $derived(MARGIN + (BOX - fieldH) / 2);

  const toX = (x: number) => VIEW / 2 + x * scale;
  const toY = (y: number) => MARGIN + BOX / 2 - y * scale;
  const deg = (rad: number) => (-rad * 180) / Math.PI;

  const quarters = [0.25, 0.5, 0.75];

  // Same color per robot as the camera view.
  const robots = $derived(
    (tracks?.robots ?? []).map((r, _i, all) => ({
      ...r,
      color: trackColor(r, all),
    })),
  );
</script>

<div class="topdown">
  <svg viewBox="0 0 {VIEW} {VIEW}" role="img" aria-label="Top-down field view with robot tracks">
    <rect class="field" x={left} y={top} width={fieldW} height={fieldH} />
    {#each quarters as q (q)}
      <line class="grid" x1={left + fieldW * q} y1={top} x2={left + fieldW * q} y2={top + fieldH} />
      <line class="grid" x1={left} y1={top + fieldH * q} x2={left + fieldW} y2={top + fieldH * q} />
    {/each}

    {#each robots as robot (robot.id)}
      {#if robot.ours}
        <g
          class="ours"
          class:stale={robot.stale}
          style="--c: {robot.color}"
          transform="translate({toX(robot.x)} {toY(robot.y)}) rotate({deg(robot.yaw)})"
        >
          <rect x="-12" y="-9" width="24" height="18" />
          <line x1="10" y1="-9" x2="10" y2="9" />
        </g>
      {:else}
        <g class="opp" class:stale={robot.stale} style="--c: {robot.color}">
          <circle cx={toX(robot.x)} cy={toY(robot.y)} r="11" />
          <circle class="core" cx={toX(robot.x)} cy={toY(robot.y)} r="2.5" />
        </g>
      {/if}
      <text
        class="name mono"
        class:stale={robot.stale}
        x={toX(robot.x)}
        y={toY(robot.y) - 15}
        style="--c: {robot.color}">{displayLabel(robot.label)}</text
      >
    {/each}

    <path class="cam" d="M 120 236 L 112 228 L 128 228 Z" />
  </svg>
  {#if showLabel}<span class="tag">TOP DOWN</span>{/if}
  {#if !hasField}<span class="empty mono">NO FIELD</span>{/if}
  <span class="caption mono">▲ cam</span>
</div>

<style>
  .topdown {
    position: relative;
    width: 100%;
    aspect-ratio: 1;
    border: 2px solid var(--ink);
    background: var(--bg);
  }
  svg {
    display: block;
    width: 100%;
    height: 100%;
  }
  .field {
    fill: none;
    stroke: var(--ink);
    stroke-width: 1.5;
  }
  .grid {
    stroke: var(--grid);
    stroke-width: 1;
  }
  .ours rect {
    fill: var(--c);
    stroke: var(--ink);
    stroke-width: 1;
  }
  .ours line {
    stroke: var(--ink);
    stroke-width: 3;
  }
  .opp circle {
    fill: none;
    stroke: var(--c);
    stroke-width: 3;
  }
  .opp .core {
    fill: var(--c);
    stroke: none;
  }
  /* Colored text with a background-colored halo, so it reads on the grid and in both themes. */
  .name {
    font-size: 8px;
    font-weight: 600;
    text-anchor: middle;
    fill: var(--c);
    stroke: var(--bg);
    stroke-width: 2.5px;
    paint-order: stroke;
  }
  .stale {
    opacity: 0.35;
  }
  .cam {
    fill: var(--ink);
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
  .empty {
    position: absolute;
    inset: 0;
    display: flex;
    align-items: center;
    justify-content: center;
    color: var(--muted);
    font-size: 13px;
    letter-spacing: 0.1em;
  }
  .caption {
    position: absolute;
    right: 6px;
    bottom: 4px;
    font-size: 10px;
    color: var(--muted);
  }
</style>
