<script lang="ts">
  // The worst diagnostics module, linking to its detail on the Diagnostics tab.
  import { LEVEL_NAMES, diagnostics } from "../lib/diagnostics.svelte";
  import { ui } from "../lib/ui.svelte";

  let { compact = false }: { compact?: boolean } = $props();

  const worst = $derived(diagnostics.sorted[0]);
  const bad = $derived(!!worst && worst.level !== 0);
  const levelClass = $derived(worst ? LEVEL_NAMES[worst.level].toLowerCase() : "ok");
</script>

<button
  class="strip"
  class:compact
  class:bad
  onclick={() => (worst && bad ? ui.showModule(worst.id) : (ui.tab = "diagnostics"))}
>
  {#if worst && bad}
    <span class="level-badge {levelClass}">{LEVEL_NAMES[worst.level]}</span>
    <span class="what">
      <span class="mono id">{worst.id}</span>
      <span class="msg muted">{worst.message}</span>
    </span>
  {:else}
    <span class="level-badge">OK</span>
    <span class="what">
      <span class="msg muted"
        >{diagnostics.modules.length
          ? `All ${diagnostics.modules.length} modules OK`
          : "No diagnostics yet"}</span
      >
    </span>
  {/if}
  <span class="go">{compact ? "→" : "DIAGNOSTICS →"}</span>
</button>

<style>
  .strip {
    display: flex;
    align-items: center;
    gap: 12px;
    width: 100%;
    min-height: 48px;
    padding: 0 14px;
    border: 2px solid var(--ink);
    background: transparent;
    color: var(--ink);
    text-align: left;
  }
  .strip.bad {
    border-color: var(--amber);
  }
  .level-badge {
    width: auto;
    padding: 3px 7px;
    flex-shrink: 0;
  }
  .what {
    flex-grow: 1;
    min-width: 0;
    display: flex;
    align-items: baseline;
    gap: 12px;
  }
  .id {
    font-size: 14px;
    font-weight: 600;
    white-space: nowrap;
  }
  .msg {
    font-size: 14px;
    min-width: 0;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .go {
    flex-shrink: 0;
    font-weight: 700;
    font-size: 13px;
    letter-spacing: 0.06em;
  }
  .compact {
    min-height: 56px;
    gap: 10px;
    padding: 0 12px;
  }
  .compact .what {
    flex-direction: column;
    align-items: flex-start;
    gap: 4px;
  }
  .compact .id {
    font-size: 13px;
  }
  .compact .msg {
    font-size: 12px;
    max-width: 100%;
  }
  .compact .go {
    font-size: 16px;
  }
</style>
