<script lang="ts">
  import { diagnostics } from "../lib/diagnostics.svelte";
  import { ui, type Tab } from "../lib/ui.svelte";

  let { wide }: { wide: boolean } = $props();

  const alerts = $derived(diagnostics.modules.filter((m) => m.level !== 0).length);
  const tabs: { id: Tab; wide: string; narrow: string }[] = [
    { id: "main", wide: "MAIN", narrow: "MAIN" },
    { id: "diagnostics", wide: "DIAGNOSTICS", narrow: "DIAG" },
    { id: "system", wide: "SYSTEM", narrow: "SYSTEM" },
  ];
</script>

<nav aria-label="Sections" class:wide>
  {#each tabs as tab (tab.id)}
    <button aria-pressed={ui.tab === tab.id} onclick={() => (ui.tab = tab.id)}>
      {wide ? tab.wide : tab.narrow}
      {#if tab.id === "diagnostics" && alerts > 0}
        <span class="badge">{alerts}</span>
      {/if}
    </button>
  {/each}
</nav>

<style>
  nav {
    display: grid;
    grid-template-columns: repeat(3, minmax(0, 1fr));
    gap: 6px;
    padding: 12px 16px;
  }
  nav.wide {
    display: flex;
    gap: 4px;
    padding: 0;
    margin-left: 12px;
  }
  button {
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 6px;
    height: 44px;
    padding: 0 10px;
    border: 2px solid var(--ink);
    background: transparent;
    color: var(--ink);
    font-weight: 700;
    font-size: 12px;
    letter-spacing: 0.06em;
  }
  .wide button {
    gap: 8px;
    height: 44px;
    padding: 0 14px;
    font-size: 13px;
  }
  button[aria-pressed="true"] {
    background: var(--ink);
    color: var(--bg);
  }
  .badge {
    min-width: 16px;
    height: 16px;
    padding: 0 3px;
    display: flex;
    align-items: center;
    justify-content: center;
    background: var(--amber);
    color: var(--on-amber);
    font-family: var(--mono);
    font-size: 10px;
    font-weight: 600;
  }
  .wide .badge {
    min-width: 18px;
    height: 18px;
    padding: 0 4px;
    font-size: 11px;
  }
</style>
