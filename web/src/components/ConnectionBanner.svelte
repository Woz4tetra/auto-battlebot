<script lang="ts">
  // Three states: relay unreachable, relay up but the app is silent, and connected (hidden).
  import { APP_TIMEOUT_MS, status } from "../lib/status.svelte";
  import { relayUrl } from "../lib/connection";

  const state = $derived(
    status.connection !== "open" ? "relay" : status.appUp ? "ok" : ("app" as const),
  );
  const since = $derived(
    status.systemAt ? `${Math.round((status.now - status.systemAt) / 1000)} s ago` : "not yet",
  );
</script>

{#if state === "relay"}
  <div class="banner error" role="alert">
    <span class="tag">OFFLINE</span>
    <span class="text"
      >Relay unreachable at <span class="mono">{relayUrl()}</span>. Reconnecting.</span
    >
  </div>
{:else if state === "app"}
  <div class="banner warn" role="alert">
    <span class="tag">APP DOWN</span>
    <span class="text"
      >Relay is up, but the app has sent no status for {APP_TIMEOUT_MS / 1000} s. Last status:
      {since}.</span
    >
  </div>
{/if}

<style>
  .banner {
    display: flex;
    align-items: center;
    gap: 12px;
    min-height: 44px;
    padding: 8px 16px;
    border-bottom: 2px solid var(--ink);
    font-size: 14px;
    font-weight: 600;
  }
  .tag {
    flex-shrink: 0;
    padding: 3px 7px;
    font-family: var(--mono);
    font-size: 11px;
    font-weight: 600;
    letter-spacing: 0.06em;
  }
  .error .tag {
    background: var(--red);
    color: #ffffff;
  }
  .warn .tag {
    background: var(--amber);
    color: var(--on-amber);
  }
  .text {
    min-width: 0;
    overflow-wrap: anywhere;
  }
</style>
