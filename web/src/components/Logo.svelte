<script lang="ts">
  let { width = 50 }: { width?: number } = $props();
</script>

<!-- The logo from logo/bwbots_logo.svg, colored by theme tokens so dark mode inverts it. -->
<svg
  {width}
  height={Math.round((width * 290) / 370)}
  viewBox="55 40 370 290"
  role="img"
  aria-label="BW bots logo"
>
  <path class="left" d="M202 47 A137 137 0 0 0 202 321 L202 240 A56 56 0 0 1 202 128 Z" />
  <path class="right" d="M277 47 A137 137 0 0 1 277 321 L277 240 A56 56 0 0 0 277 128 Z" />
  <rect class="pill" x="130" y="118" width="217" height="132" rx="66" />
  <text x="238.5" y="222" text-anchor="middle">BW</text>
</svg>

<style>
  svg {
    flex-shrink: 0;
    display: block;
  }
  .left,
  .right {
    stroke: var(--ink);
    stroke-width: 7;
    stroke-linejoin: round;
  }
  .left {
    fill: var(--logo-left);
  }
  .right {
    fill: var(--logo-right);
  }
  .pill {
    fill: var(--logo-pill);
  }
  text {
    fill: var(--logo-text);
    font-family: var(--sans);
    font-size: 104px;
    font-weight: 800;
  }
</style>
