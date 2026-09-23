<script lang="ts">
  // Module list with worst level, and the selected module's plot and values.
  import { LEVEL_NAMES, diagnostics, type DiagModule } from "../lib/diagnostics.svelte";
  import { status } from "../lib/status.svelte";
  import { ui } from "../lib/ui.svelte";
  import { formatDuration } from "../lib/time";
  import DiagPlot from "./DiagPlot.svelte";

  let { wide }: { wide: boolean } = $props();

  const list = $derived(diagnostics.sorted);
  const selected = $derived<DiagModule | undefined>(
    (ui.selectedModule && diagnostics.get(ui.selectedModule)) || list[0],
  );
  const counts = $derived({
    warn: list.filter((m) => m.level === 1).length,
    error: list.filter((m) => m.level === 2).length,
  });

  // Plotted value per module; defaults to the module's first numeric value.
  let plotKeys = $state<Record<string, string>>({});
  const plotKey = $derived.by(() => {
    if (!selected) return null;
    const chosen = plotKeys[selected.id];
    if (chosen && selected.values.some((v) => v.key === chosen)) return chosen;
    return selected.values.find((v) => typeof v.value === "number")?.key ?? null;
  });

  // The runner module logs uptime_s and autonomy_on_s too, so they can be plotted; these read
  // /status/system so they tick at 10 Hz instead of the diagnostics rate.
  const timers = $derived.by(() => {
    const sys = status.appUp ? status.system : null;
    return [
      { label: "UPTIME", value: sys ? formatDuration(sys.uptime_s) : "--" },
      {
        label: "RADIO SWITCH",
        value: !sys
          ? "--"
          : sys.autonomy_switch_on === undefined
            ? "NONE"
            : sys.autonomy_switch_on
              ? formatDuration(sys.autonomy_on_s)
              : "OFF",
      },
    ];
  });

  const levelClass = (m: DiagModule) => LEVEL_NAMES[m.level].toLowerCase();
  const show = (v: number | string | null) =>
    v === null
      ? "null"
      : typeof v === "number"
        ? Number.isInteger(v)
          ? String(v)
          : v.toFixed(3)
        : v;
  const ago = (m: DiagModule) =>
    `${Math.max(0, (status.now - m.updatedAt) / 1000).toFixed(1)} s ago`;
</script>

<div class="diag" class:wide>
  <aside>
    <div class="timers">
      {#each timers as t (t.label)}
        <div class="timer">
          <span class="label">{t.label}</span>
          <span class="mono tval">{t.value}</span>
        </div>
      {/each}
    </div>
    <div class="list-head">
      <span class="label">MODULES</span>
      <span class="mono muted small">{list.length} · {counts.warn} warn · {counts.error} error</span
      >
    </div>
    {#each list as m (m.id)}
      <button
        class="module"
        aria-pressed={selected?.id === m.id}
        onclick={() => (ui.selectedModule = m.id)}
      >
        <span class="level-badge {levelClass(m)}">{LEVEL_NAMES[m.level]}</span>
        <span class="names">
          <span class="mono id">{m.id}</span>
          <span class="msg">{m.message || " "}</span>
        </span>
      </button>
    {:else}
      <div class="none muted">No diagnostics received yet.</div>
    {/each}
  </aside>

  <main>
    {#if selected}
      <div class="title">
        <span class="mono id">{selected.id}</span>
        <span class="level-badge {levelClass(selected)}">{LEVEL_NAMES[selected.level]}</span>
        {#if wide}
          <span class="grow"></span>
          <span class="mono muted small">updated {ago(selected)} · /diagnostics/{selected.id}</span>
        {/if}
      </div>
      {#if selected.message}<div class="message">{selected.message}</div>{/if}

      {#if plotKey}
        <DiagPlot module={selected.id} valueKey={plotKey} height={wide ? 132 : 104} />
      {/if}

      <div class="values">
        {#if wide}<span class="label section-head">VALUES</span>{/if}
        {#each selected.values as v (v.key)}
          {@const numeric = typeof v.value === "number"}
          <button
            class="leader value"
            class:plotted={v.key === plotKey}
            disabled={!numeric}
            title={numeric ? "Plot this value" : undefined}
            onclick={() => (plotKeys = { ...plotKeys, [selected.id]: v.key })}
          >
            <span class="k mono">{v.key}</span><span class="dots"></span><span class="v"
              >{show(v.value)}</span
            >
          </button>
        {/each}
      </div>
    {/if}
  </main>
</div>

<style>
  .diag {
    display: flex;
    flex-direction: column;
  }
  .diag.wide {
    flex-direction: row;
    flex-grow: 1;
    min-height: 0;
  }
  aside {
    display: flex;
    flex-direction: column;
  }
  .wide aside {
    width: 380px;
    flex-shrink: 0;
    border-right: 2px solid var(--ink);
    overflow-y: auto;
  }
  .timers {
    display: grid;
    grid-template-columns: 1fr 1fr;
    border-bottom: 2px solid var(--ink);
  }
  .timer {
    display: flex;
    flex-direction: column;
    gap: 4px;
    padding: 10px 16px;
  }
  .timer + .timer {
    border-left: 1px solid var(--ink);
  }
  .wide .timer {
    padding: 12px 28px;
  }
  .tval {
    font-size: 20px;
    font-weight: 600;
  }
  .list-head {
    display: flex;
    align-items: center;
    justify-content: space-between;
    height: 40px;
    padding: 0 16px;
    border-bottom: 2px solid var(--ink);
  }
  .wide .list-head {
    height: 64px;
    padding: 0 28px;
    flex-shrink: 0;
  }
  .small {
    font-size: 12px;
  }
  .module {
    display: flex;
    align-items: center;
    gap: 12px;
    flex-shrink: 0;
    height: 60px;
    padding: 0 16px;
    border: none;
    border-bottom: 1px solid var(--ink);
    background: transparent;
    color: var(--ink);
    text-align: left;
  }
  .wide .module {
    gap: 14px;
    height: 66px;
    padding: 0 28px;
  }
  .module[aria-pressed="true"] {
    background: var(--ink);
    color: var(--bg);
  }
  .module .level-badge {
    width: 50px;
    flex-shrink: 0;
  }
  .wide .module .level-badge {
    width: 52px;
  }
  .names {
    display: flex;
    flex-direction: column;
    gap: 5px;
    min-width: 0;
  }
  .names .id {
    font-size: 14px;
    font-weight: 600;
    line-height: 1;
  }
  .names .msg {
    font-size: 12px;
    line-height: 1.1;
    color: var(--muted);
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }
  .module[aria-pressed="true"] .msg {
    color: var(--bg);
  }
  .none {
    padding: 16px;
    font-size: 14px;
  }

  main {
    display: flex;
    flex-direction: column;
    gap: 14px;
    padding: 20px 16px;
    min-width: 0;
  }
  .wide main {
    flex-grow: 1;
    gap: 20px;
    padding: 26px 28px;
    overflow-y: auto;
  }
  .title {
    display: flex;
    align-items: center;
    gap: 10px;
    min-width: 0;
  }
  .wide .title {
    gap: 14px;
  }
  .title .id {
    font-weight: 600;
    font-size: 18px;
    overflow-wrap: anywhere;
  }
  .wide .title .id {
    font-size: 24px;
  }
  .title .level-badge {
    padding: 3px 7px;
  }
  .grow {
    flex-grow: 1;
  }
  .message {
    font-size: 16px;
    font-weight: 600;
  }
  .wide .message {
    font-size: 18px;
  }
  .values {
    display: flex;
    flex-direction: column;
  }
  .value {
    width: 100%;
    padding: 0;
    border: none;
    background: transparent;
    color: var(--ink);
    text-align: left;
    align-items: center;
  }
  .value:disabled {
    color: var(--ink);
  }
  .value .k {
    font-weight: 400;
    font-size: 13px;
    overflow: hidden;
    text-overflow: ellipsis;
  }
  .wide .value .k {
    font-size: 14px;
  }
  .value.plotted .k {
    text-decoration: underline;
    text-underline-offset: 3px;
  }
</style>
