<script lang="ts">
  import type { ResolvedConfigField } from "../generated/protocol";
  import { status } from "../lib/status.svelte";

  let filter = $state("");
  let hideDefaults = $state(false);

  const sections = $derived.by(() => {
    const needle = filter.trim().toLowerCase();
    const grouped = new Map<string, ResolvedConfigField[]>();
    for (const field of status.config?.fields ?? []) {
      if (hideDefaults && field.source === "default") continue;
      const name = field.section || "(top level)";
      if (needle && !`${name}.${field.key}`.toLowerCase().includes(needle)) continue;
      const rows = grouped.get(name) ?? [];
      rows.push(field);
      grouped.set(name, rows);
    }
    return [...grouped.entries()];
  });
</script>

<div class="config">
  <span class="label section-head">RESOLVED CONFIG</span>
  {#if !status.config}
    <span class="muted">Waiting for the app.</span>
  {:else}
    <div class="tools">
      <input class="filter" type="search" placeholder="Filter section or key" bind:value={filter} />
      <label class="muted"
        ><input type="checkbox" bind:checked={hideDefaults} /> Hide defaults</label
      >
    </div>
    {#each sections as [name, fields] (name)}
      <div class="section">
        <span class="name">[{name}]</span>
        {#each fields as field (field.key)}
          <div class="row">
            <span class="k">{field.key}</span>
            <span class="v">{field.value || "unset"}</span>
            <span class="src" class:default={field.source === "default"}>{field.source}</span>
          </div>
        {/each}
      </div>
    {/each}
  {/if}
</div>

<style>
  .config {
    display: flex;
    flex-direction: column;
    gap: 10px;
    min-width: 0;
  }
  .tools {
    display: flex;
    flex-wrap: wrap;
    gap: 12px;
    align-items: center;
  }
  .filter {
    flex: 1 1 200px;
    min-width: 0;
    font: inherit;
    padding: 6px 8px;
    border: 2px solid var(--ink);
    background: var(--bg);
    color: var(--ink);
  }
  .section {
    display: flex;
    flex-direction: column;
    font-family: var(--mono);
    font-size: 12px;
  }
  .name {
    font-weight: 600;
    padding: 6px 0 2px;
  }
  .row {
    display: grid;
    grid-template-columns: minmax(0, 2fr) minmax(0, 2fr) minmax(0, 1.5fr);
    gap: 8px;
    padding: 2px 0;
    border-bottom: 1px solid var(--grid);
  }
  .k,
  .v,
  .src {
    overflow-wrap: anywhere;
  }
  .src {
    color: var(--muted);
  }
  .src.default {
    color: var(--faint);
  }
</style>
