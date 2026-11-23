<script lang="ts">
  import { PLATFORM, pipelineLayout } from "../lib/pipeline";
  import type { Feature, Stage } from "../lib/types";

  let {
    stages,
    features,
    selected,
    diffOverlay,
    onpick,
    onselect,
  }: {
    stages: Stage[];
    features: Feature[];
    selected: string | null;
    diffOverlay: boolean;
    onpick: (stage: string) => void;
    onselect: (feature: string) => void;
  } = $props();
  const layout = $derived(pipelineLayout(stages, features));
  const platform = $derived(stages.find((s) => s.id === PLATFORM));
  const rows = $derived(layout.bands.length ? layout.bands.length + 3 : 2);
  const outsideCount = $derived(layout.outside.reduce((n, g) => n + g.features.length, 0));
</script>

{#snippet counts(stage: Stage)}
  <span class="counts">
    <span title="proven"><i class="dot passing"></i>{stage.proven}</span>
    <span title="stale or never run"><i class="dot stale"></i>{stage.stale}</span>
    <span title="failed"><i class="dot failed"></i>{stage.failed}</span>
    <span title="no proof yet"><i class="dot gap"></i>{stage.gap}</span>
  </span>
{/snippet}

{#snippet chip(feature: Feature)}
  <button
    class="chip {feature.state}"
    class:selected={feature.key === selected}
    class:diff={diffOverlay && feature.in_diff}
    title={`${feature.name} (${feature.state})`}
    onclick={() => onselect(feature.key)}
  >
    <i class="dot {feature.state}"></i><span class="name">{feature.name}</span>
  </button>
{/snippet}

<div class="overview">
  <div class="lanes" style={`--lanes: ${layout.lanes.length}`}>
    {#each layout.lanes as lane, i (lane.stage.id)}
      <div class="lane" style={`grid-column: ${i + 1}; grid-row: 1 / span ${rows}`}></div>
      <button class="stage" style={`grid-column: ${i + 1}`} onclick={() => onpick(lane.stage.id)}>
        <span class="label">{lane.stage.label}</span>
        <span class="total">{lane.stage.features.length}</span>
        {@render counts(lane.stage)}
      </button>
      <div class="chips" style={`grid-column: ${i + 1}`}>
        {#each lane.chips as feature (feature.key)}{@render chip(feature)}{/each}
      </div>
    {/each}
    {#if layout.bands.length}
      <h4 class="bandhead">Across stages</h4>
    {/if}
    {#each layout.bands as band, k (band.feature.key)}
      <div
        class="band"
        style={`grid-column: ${band.start + 1} / ${band.end + 2}; grid-row: ${k + 4}`}
      >
        {@render chip(band.feature)}
      </div>
    {/each}
  </div>

  {#if platform}
    <section class="outside">
      <button class="stage" onclick={() => onpick(PLATFORM)}>
        <span class="label">{platform.label}</span>
        <span class="total">{outsideCount} outside the loop</span>
        {@render counts(platform)}
      </button>
      {#each layout.outside as group (group.kind)}
        <div class="group">
          <h4 class="mono">{group.kind}</h4>
          <div class="flow">
            {#each group.features as feature (feature.key)}{@render chip(feature)}{/each}
          </div>
        </div>
      {/each}
    </section>
  {/if}

  <p class="legend">
    <span><i class="dot passing"></i>proven</span>
    <span><i class="dot stale"></i>stale or never run</span>
    <span><i class="dot failed"></i>failed</span>
    <span><i class="dot gap"></i>no proof yet</span>
    <span>click a stage to open its map, a feature for its details</span>
  </p>
</div>

<style>
  .overview {
    flex: 1;
    min-width: 0;
    padding: 20px 24px;
    overflow: auto;
  }
  .lanes {
    display: grid;
    grid-template-columns: repeat(var(--lanes), minmax(150px, 1fr));
    grid-auto-rows: auto;
    gap: 6px 8px;
    min-width: calc(var(--lanes) * 158px);
  }
  .lane {
    background: color-mix(in srgb, var(--line) 45%, var(--bg));
    border-radius: 8px;
  }
  .stage {
    grid-row: 1;
    display: flex;
    flex-wrap: wrap;
    align-items: baseline;
    gap: 4px 8px;
    margin: 6px;
    padding: 8px 10px;
    border-radius: 8px;
    background: var(--panel);
    text-align: left;
  }
  .label {
    font-weight: 600;
    font-size: 15px;
  }
  .total {
    color: var(--muted);
    font-size: 12px;
  }
  .counts {
    display: flex;
    gap: 8px;
    width: 100%;
    font-size: 12px;
    font-variant-numeric: tabular-nums;
  }
  .chips {
    grid-row: 2;
    display: flex;
    flex-direction: column;
    gap: 4px;
    padding: 0 6px 6px;
  }
  .bandhead {
    grid-column: 1 / -1;
    grid-row: 3;
    margin: 10px 8px 0;
    color: var(--muted);
    font-size: 12px;
    font-weight: 600;
    position: relative;
  }
  .band {
    padding: 0 6px;
    position: relative;
  }
  .band .chip {
    width: 100%;
  }
  .chip {
    display: flex;
    align-items: center;
    min-width: 0;
    padding: 3px 8px;
    border-radius: 4px;
    border-left-width: 3px;
    background: var(--panel);
    font-size: 12px;
    text-align: left;
  }
  .chip.gap {
    border-style: dashed;
    border-color: var(--gap);
  }
  .chip.failed {
    border-left-color: var(--failed);
  }
  .chip.selected {
    outline: 2px solid var(--accent);
    outline-offset: 1px;
  }
  .chip.diff {
    background: color-mix(in srgb, var(--diff) 25%, var(--panel));
  }
  .name {
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .outside {
    margin-top: 20px;
    display: grid;
    gap: 10px;
  }
  .outside .stage {
    justify-self: start;
    margin: 0;
  }
  .group h4 {
    margin: 0 0 4px;
    color: var(--muted);
    font-weight: 500;
  }
  .flow {
    display: flex;
    flex-wrap: wrap;
    gap: 4px;
  }
  .legend {
    display: flex;
    flex-wrap: wrap;
    gap: 16px;
    margin-top: 24px;
    color: var(--muted);
  }
</style>
