<script lang="ts">
  import Graph from "./components/Graph.svelte";
  import Overview from "./components/Overview.svelte";
  import SidePanel from "./components/SidePanel.svelte";
  import CopyButton from "./components/CopyButton.svelte";
  import { buildElements, type View } from "./lib/graph";
  import { live } from "./lib/live.svelte";

  type Tab = "overview" | View;

  let tab = $state<Tab>("overview");
  let stage = $state<string | null>(null);
  let selected = $state<string | null>(null);
  let diffOverlay = $state(false);
  let showInfrastructure = $state(false);
  let expanded = $state(new Set<string>());

  const map = $derived(live.map);
  const elements = $derived(
    map && tab !== "overview"
      ? buildElements(map, { view: tab, expanded, stage, diffOverlay, showInfrastructure })
      : { nodes: [], edges: [] },
  );
  const feature = $derived(map?.features.find((f) => f.key === selected) ?? null);

  function toggle(group: string): void {
    const next = new Set(expanded);
    if (next.has(group)) next.delete(group);
    else next.add(group);
    expanded = next;
  }

  function pickStage(id: string): void {
    stage = id;
    tab = "map";
  }

  $effect(() => {
    if (diffOverlay && live.wouldRun === null) void live.loadWouldRun();
  });
</script>

<div class="shell">
  <nav>
    <strong>Feature map</strong>
    <button class:active={tab === "overview"} onclick={() => (tab = "overview")}>Overview</button>
    <button class:active={tab === "map"} onclick={() => (tab = "map")}>Map</button>
    <button class:active={tab === "impact"} onclick={() => (tab = "impact")}>Impact</button>
    {#if stage}
      <button onclick={() => (stage = null)}>stage: {stage} ×</button>
    {/if}
    <button class:active={diffOverlay} onclick={() => (diffOverlay = !diffOverlay)}>
      diff overlay
    </button>
    {#if tab === "impact" && map}
      <button
        class:active={showInfrastructure}
        title={map.impact.infrastructure.join(", ")}
        onclick={() => (showInfrastructure = !showInfrastructure)}
      >
        {showInfrastructure ? "collapse" : "expand"}
        {map.impact.infrastructure.filter((m) => !m.endsWith("/*")).length} infrastructure modules
      </button>
    {/if}
    <span class="spacer"></span>
    {#if map}
      <span class="meta mono">
        {map.head}{map.dirty ? `+${map.dirty.slice(0, 8)}` : ""} · {map.features.length} features ·
        {map.check.errors.length} errors · {map.check.gaps.length} gaps
        {#if map.runs.length}· last run {map.runs[map.runs.length - 1].stamp}{/if}
      </span>
    {/if}
    <span class="live" class:on={live.connected} title="live updates">
      {live.connected ? "live" : "offline"}
    </span>
  </nav>

  {#if diffOverlay && map}
    <div class="diffbar">
      <span>{map.diff.files.length} changed files touch {map.diff.features.length} features.</span>
      {#if live.wouldRun}
        <span class="mono">would run: {live.wouldRun.join(" · ") || "nothing"}</span>
      {:else}
        <span class="muted">working out what --changed would run…</span>
      {/if}
      <CopyButton text={map.diff.command} label="copy command" />
    </div>
  {/if}

  {#if live.error}<p class="error">{live.error}</p>{/if}
  {#if map && map.check.errors.length}
    <details class="errors">
      <summary>{map.check.errors.length} registry errors</summary>
      <ul class="mono">
        {#each map.check.errors as error (error)}<li>{error}</li>{/each}
      </ul>
    </details>
  {/if}

  <main>
    {#if !map}
      <p class="loading">Loading the map…</p>
    {:else if tab === "overview"}
      <Overview
        stages={map.stages}
        features={map.features}
        {selected}
        {diffOverlay}
        onpick={pickStage}
        onselect={(key) => (selected = key)}
      />
    {:else}
      <div class="canvas">
        <Graph {elements} {selected} onselect={(key) => (selected = key)} ontoggle={toggle} />
      </div>
    {/if}
    {#if feature}
      <SidePanel {feature} onclose={() => (selected = null)} />
    {/if}
  </main>
</div>

<style>
  .shell {
    display: flex;
    flex-direction: column;
    height: 100%;
  }
  nav {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    gap: 8px;
    padding: 10px 16px;
    border-bottom: 1px solid var(--line);
    background: var(--panel);
  }
  .spacer {
    flex: 1;
  }
  .meta,
  .muted {
    color: var(--muted);
  }
  .live {
    font-size: 12px;
    color: var(--failed);
  }
  .live.on {
    color: var(--passing);
  }
  .diffbar {
    display: flex;
    flex-wrap: wrap;
    gap: 12px;
    align-items: center;
    padding: 8px 16px;
    border-bottom: 1px solid var(--line);
    background: color-mix(in srgb, var(--diff) 18%, var(--panel));
  }
  .errors,
  .error {
    margin: 0;
    padding: 8px 16px;
    color: var(--failed);
  }
  main {
    flex: 1;
    display: flex;
    min-height: 0;
  }
  .canvas {
    flex: 1;
    min-width: 0;
  }
  .loading {
    padding: 24px;
  }
</style>
