<script lang="ts">
  // Fires `onconfirm` only after the press is held for `holdMs`. Releasing early cancels.
  import type { Snippet } from "svelte";

  let {
    holdMs = 800,
    danger = false,
    filled = false,
    disabled = false,
    height = 52,
    onconfirm,
    children,
  }: {
    holdMs?: number;
    danger?: boolean;
    filled?: boolean;
    disabled?: boolean;
    height?: number;
    onconfirm: () => void;
    children: Snippet;
  } = $props();

  let progress = $state(0);
  let raf = 0;
  let startedAt = 0;

  function tick(now: number) {
    progress = Math.min(1, (now - startedAt) / holdMs);
    if (progress >= 1) {
      raf = 0;
      progress = 0;
      onconfirm();
      return;
    }
    raf = requestAnimationFrame(tick);
  }

  function start(e: Event) {
    if (disabled || raf) return;
    e.preventDefault();
    startedAt = performance.now();
    raf = requestAnimationFrame(tick);
  }

  function cancel() {
    if (raf) cancelAnimationFrame(raf);
    raf = 0;
    progress = 0;
  }

  function keydown(e: KeyboardEvent) {
    if ((e.key === " " || e.key === "Enter") && !e.repeat) start(e);
  }
  function keyup(e: KeyboardEvent) {
    if (e.key === " " || e.key === "Enter") cancel();
  }
</script>

<button
  class:danger
  class:filled
  style:height="{height}px"
  {disabled}
  onpointerdown={start}
  onpointerup={cancel}
  onpointerleave={cancel}
  onpointercancel={cancel}
  oncontextmenu={(e) => e.preventDefault()}
  onkeydown={keydown}
  onkeyup={keyup}
  onblur={cancel}
>
  {@render children()}
  <span class="bar" style:width="{progress * 100}%"></span>
</button>

<style>
  button {
    position: relative;
    overflow: hidden;
    width: 100%;
    border: 2px solid var(--ink);
    background: transparent;
    color: var(--ink);
    font-weight: 700;
    font-size: 14px;
    letter-spacing: 0.06em;
    user-select: none;
    -webkit-user-select: none;
    -webkit-touch-callout: none;
    touch-action: none;
  }
  .filled {
    background: var(--ink);
    color: var(--bg);
  }
  .danger {
    border-color: var(--red);
    color: var(--red);
  }
  .bar {
    position: absolute;
    left: 0;
    bottom: 0;
    height: 4px;
    background: var(--red);
  }
  .filled .bar {
    background: var(--amber);
  }
  button:not(.danger):not(.filled) .bar {
    background: var(--ink);
  }
  button:disabled {
    border-color: var(--faint);
    background: transparent;
    color: var(--faint);
  }
</style>
