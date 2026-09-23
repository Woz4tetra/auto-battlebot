<script lang="ts">
  // Control rows 01 to 06: autonomy, opponent count, recording, field, radio mode, sticks.
  import { diagnostics } from "../lib/diagnostics.svelte";
  import { status } from "../lib/status.svelte";
  import Toggle from "./Toggle.svelte";

  let { compact = false }: { compact?: boolean } = $props();

  const sys = $derived(status.system);
  const live = $derived(status.appUp && sys !== null);

  // Which command is waiting on its ack, so its control can't be double-sent.
  let busy = $state<string | null>(null);

  async function run(key: string, fn: () => Promise<unknown>) {
    if (busy) return;
    busy = key;
    try {
      await fn();
    } finally {
      busy = null;
    }
  }

  const radioMode = $derived.by(() => {
    const v = diagnostics.findValue("behavior_mode");
    return typeof v === "string" && v ? v.replace(/_/g, " ").toUpperCase() : "--";
  });

  function bar(v: number | undefined) {
    const x = Math.max(-1, Math.min(1, v ?? 0));
    return { left: 50 + Math.min(0, x) * 50, width: Math.abs(x) * 50, x };
  }
  const lin = $derived(bar(status.sticks?.linear));
  const ang = $derived(bar(status.sticks?.angular));
  const fmt = (x: number) => (x >= 0 ? "+" : "−") + Math.abs(x).toFixed(2);
</script>

<div class="rows" class:compact>
  <div class="row">
    <span class="rownum">01</span>
    <span class="body">
      <span class="label">AUTONOMY</span>
      <span class="state">{sys ? (sys.autonomy_enabled ? "ON" : "OFF") : "--"}</span>
    </span>
    <Toggle
      label="Autonomy"
      on={sys?.autonomy_enabled ?? false}
      disabled={!live || busy !== null}
      onchange={(enabled) =>
        run("autonomy", () => status.command("/command/set_autonomy", { enabled }))}
    />
  </div>

  <div class="row">
    <span class="rownum">02</span>
    <span class="body">
      <span class="label">OPPONENTS</span>
      <span class="state">{sys?.selected_opponent_count ?? "--"}</span>
    </span>
    <div class="seg" role="group" aria-label="Opponent count">
      {#each [1, 2, 3] as count (count)}
        <button
          class="opp"
          aria-pressed={sys?.selected_opponent_count === count}
          disabled={!live || busy !== null}
          onclick={() =>
            run("opponents", () => status.command("/command/set_opponent_count", { count }))}
          >{count}</button
        >
      {/each}
    </div>
  </div>

  <div class="row">
    <span class="rownum">03</span>
    <span class="body">
      <span class="label">RECORDING</span>
      <span class="state rec">
        <span class="recdot" class:on={status.recording}></span>
        {sys ? (status.recording ? "ON" : "OFF") : "--"}
      </span>
    </span>
    <Toggle
      label="Recording"
      on={status.recording}
      disabled={!live || busy !== null}
      onchange={(enabled) =>
        run("recording", () => status.command("/command/set_recording", { enabled }))}
    />
  </div>

  <div class="row">
    <span class="rownum">04</span>
    <span class="body">
      <span class="label">FIELD</span>
      <span class="state">{sys ? (sys.initialized ? "READY" : "NOT READY") : "--"}</span>
    </span>
    <button
      class="btn"
      disabled={!live || busy !== null}
      onclick={() => run("field", () => status.command("/command/reinit_field", {}))}
      >RE-INIT</button
    >
  </div>

  <div class="row">
    <span class="rownum">05</span>
    <span class="body">
      <span class="label">RADIO MODE</span>
      <span class="state">{radioMode}</span>
    </span>
    <span class="label muted">SET ON RADIO</span>
  </div>

  <div class="sticks">
    <span class="sticks-head">
      <span class="rownum">06</span>
      <span class="label">STICKS · OUR ROBOT</span>
    </span>
    {#each [["LIN", lin], ["ANG", ang]] as const as [name, b] (name)}
      <div class="stick">
        <span class="label muted axis">{name}</span>
        <div class="track">
          <div class="fill" style:left="{b.left}%" style:width="{b.width}%"></div>
          <div class="zero"></div>
        </div>
        <span class="mono num">{status.sticks ? fmt(b.x) : "--"}</span>
      </div>
    {/each}
  </div>
</div>

<style>
  .rows {
    display: flex;
    flex-direction: column;
  }
  .row {
    display: flex;
    align-items: center;
    gap: 16px;
    height: 92px;
    padding: 0 28px;
    border-bottom: 1px solid var(--ink);
  }
  .compact .row {
    gap: 12px;
    height: 80px;
    padding: 0 16px;
  }
  .rownum {
    width: 20px;
    flex-shrink: 0;
  }
  .compact .rownum {
    width: 18px;
  }
  .body {
    flex-grow: 1;
    min-width: 0;
    display: flex;
    flex-direction: column;
    gap: 8px;
  }
  .state {
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }
  .rec {
    display: flex;
    align-items: center;
    gap: 10px;
  }
  .recdot {
    width: 10px;
    height: 10px;
    flex-shrink: 0;
  }
  .recdot.on {
    background: var(--red);
  }
  .opp {
    width: 46px;
  }
  .compact .opp {
    width: 44px;
  }
  .row > .label.muted {
    flex-shrink: 0;
  }

  .sticks {
    display: flex;
    flex-direction: column;
    gap: 14px;
    padding: 22px 28px;
  }
  .compact .sticks {
    padding: 18px 16px;
    border-bottom: 2px solid var(--ink);
  }
  .sticks-head {
    display: flex;
    gap: 16px;
    align-items: baseline;
  }
  .compact .sticks-head {
    gap: 12px;
  }
  .stick {
    display: flex;
    align-items: center;
    gap: 12px;
    padding-left: 36px;
  }
  .compact .stick {
    padding-left: 30px;
  }
  .axis {
    width: 36px;
    flex-shrink: 0;
  }
  .track {
    position: relative;
    flex-grow: 1;
    height: 10px;
    border: 1px solid var(--ink);
  }
  .fill {
    position: absolute;
    top: 0;
    height: 8px;
    background: var(--ink);
  }
  .zero {
    position: absolute;
    left: 50%;
    top: -5px;
    width: 1px;
    height: 18px;
    background: var(--ink);
  }
  .num {
    width: 52px;
    flex-shrink: 0;
    text-align: right;
    font-size: 14px;
    font-weight: 600;
  }
</style>
