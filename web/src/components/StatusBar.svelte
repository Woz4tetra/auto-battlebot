<script lang="ts">
  // Header metrics: loop rate, Jetson temperature, and recording time.
  import { status } from "../lib/status.svelte";

  let { strip = false }: { strip?: boolean } = $props();

  const loop = $derived(status.system ? status.system.loop_rate_hz.toFixed(1) : "--");
  const temp = $derived(
    status.system?.jetson_temperature_c !== undefined
      ? Math.round(status.system.jetson_temperature_c).toString()
      : "--",
  );
  const recTime = $derived.by(() => {
    if (!status.recording || status.recordingSince === null) return "OFF";
    const s = Math.max(0, Math.floor((status.now - status.recordingSince) / 1000));
    const mm = Math.floor(s / 60);
    return `${String(mm).padStart(2, "0")}:${String(s % 60).padStart(2, "0")}`;
  });
</script>

<div class="metrics" class:strip>
  <div class="cell">
    <span class="k">LOOP HZ</span>
    <span class="v">{loop}</span>
  </div>
  <div class="cell">
    <span class="k">TEMP °C</span>
    <span class="v">{temp}</span>
  </div>
  <div class="cell">
    <span class="k rec">{status.recording ? "● REC" : "REC"}</span>
    <span class="v">{recTime}</span>
  </div>
</div>

<style>
  .metrics {
    display: flex;
    align-items: stretch;
    height: 40px;
  }
  .cell {
    display: flex;
    flex-direction: column;
    justify-content: center;
    gap: 4px;
    padding: 0 14px;
    border-left: 1px solid var(--ink);
  }
  .k {
    font-family: var(--mono);
    font-size: 10px;
    letter-spacing: 0.1em;
    color: var(--muted);
    white-space: nowrap;
  }
  .k.rec {
    color: var(--red);
  }
  .v {
    font-weight: 700;
    font-stretch: 110%;
    font-size: 20px;
    line-height: 1;
    font-variant-numeric: tabular-nums;
  }

  .strip {
    display: grid;
    grid-template-columns: repeat(3, minmax(0, 1fr));
    height: auto;
    border-bottom: 1px solid var(--ink);
  }
  .strip .cell {
    padding: 10px 16px;
  }
  .strip .cell:first-child {
    border-left: none;
  }
  .strip .v {
    font-size: 18px;
  }
</style>
