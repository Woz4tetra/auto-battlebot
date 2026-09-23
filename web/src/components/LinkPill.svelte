<script lang="ts">
  // Link state from /healthz (CABLE / WI-FI / LOCAL), with a dot for relay and app health.
  import { status } from "../lib/status.svelte";

  let { small = false }: { small?: boolean } = $props();

  const text = $derived.by(() => {
    if (status.connection !== "open") return "OFFLINE";
    switch (status.health?.link) {
      case "cable":
        return "CABLE";
      case "wifi":
        return "WI-FI";
      case "local":
        return "LOCAL";
      default:
        return "LINK";
    }
  });
  const dot = $derived(
    status.connection !== "open" ? "var(--red)" : status.appUp ? "var(--green)" : "var(--amber)",
  );
  const title = $derived(
    status.connection !== "open"
      ? "Relay unreachable"
      : status.appUp
        ? "Connected"
        : "Relay up, app not publishing",
  );
</script>

<span class="pill" class:small {title}>
  <span class="dot" style:background={dot}></span>{text}
</span>

<style>
  .pill {
    display: flex;
    align-items: center;
    gap: 8px;
    height: 36px;
    padding: 0 12px;
    flex-shrink: 0;
    background: var(--ink);
    color: var(--bg);
    font-family: var(--mono);
    font-size: 12px;
    font-weight: 600;
    letter-spacing: 0.08em;
    white-space: nowrap;
  }
  .dot {
    width: 8px;
    height: 8px;
  }
  .small {
    height: 30px;
    gap: 6px;
    padding: 0 8px;
    font-size: 11px;
  }
  .small .dot {
    width: 7px;
    height: 7px;
  }
</style>
