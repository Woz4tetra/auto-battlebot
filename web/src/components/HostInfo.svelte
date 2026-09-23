<script lang="ts">
  // Host rows on the System tab. Narrow layout shows them as "this phone".
  import { status } from "../lib/status.svelte";

  let { wide }: { wide: boolean } = $props();

  const net = $derived(status.network);
  const link = $derived.by(() => {
    switch (status.health?.link) {
      case "cable":
        return "CABLE";
      case "wifi":
        return net?.wifi_interface ? `Wi-Fi · ${net.wifi_interface}` : "Wi-Fi";
      case "local":
        return "LOCAL";
      default:
        return "--";
    }
  });
  const relay = $derived(
    status.connection === "open"
      ? "CONNECTED"
      : status.connection === "connecting"
        ? "CONNECTING"
        : "OFFLINE",
  );

  const rows = $derived(
    wide
      ? [
          { label: "Dashboard", value: net ? `${net.hostname}.local` : "--" },
          { label: "Cable address", value: net?.cable_address || "--" },
          { label: "Relay", value: relay },
          { label: "App", value: status.appUp ? "RUNNING" : "DOWN" },
          {
            label: "Loop target",
            value: status.app ? `${status.app.max_loop_rate_hz.toFixed(0)} Hz` : "--",
          },
        ]
      : [
          { label: "Connected over", value: link },
          { label: "Box address", value: location.hostname || "--" },
          { label: "URL", value: net ? `${net.hostname}.local` : "--" },
          { label: "Relay", value: relay },
          { label: "App", value: status.appUp ? "RUNNING" : "DOWN" },
        ],
  );
</script>

<div class="host">
  <span class="label section-head">{wide ? "HOST" : "THIS DEVICE"}</span>
  {#each rows as row (row.label)}
    <div class="leader">
      <span class="k">{row.label}</span><span class="dots"></span><span class="v">{row.value}</span>
    </div>
  {/each}
  {#if !wide}
    <span class="muted note"
      >Wi-Fi access is switched on and off from the System tab over the cable.</span
    >
  {/if}
</div>

<style>
  .host {
    display: flex;
    flex-direction: column;
  }
  .note {
    padding-top: 8px;
    font-size: 12px;
  }
</style>
