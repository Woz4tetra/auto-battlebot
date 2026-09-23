<script lang="ts">
  // Reboot and power off, each behind an 800 ms hold.
  import { status } from "../lib/status.svelte";
  import HoldButton from "./HoldButton.svelte";

  let { wide }: { wide: boolean } = $props();

  let busy = $state(false);

  async function act(action: "reboot_host" | "poweroff_host") {
    if (busy) return;
    busy = true;
    try {
      await status.command("/command/system_action", { action });
    } finally {
      busy = false;
    }
  }
</script>

<section class="power" class:wide>
  <span class="intro">
    <span class="label red">POWER</span>
    <span class="muted text"
      >Stops the robot and closes the SVO and MCAP recordings first. Hold 0.8 s to confirm.</span
    >
  </span>
  <div class="buttons">
    <HoldButton disabled={busy || !status.appUp} onconfirm={() => act("reboot_host")}
      >REBOOT</HoldButton
    >
    <HoldButton danger disabled={busy || !status.appUp} onconfirm={() => act("poweroff_host")}
      >POWER OFF</HoldButton
    >
  </div>
</section>

<style>
  .power {
    display: flex;
    flex-direction: column;
    gap: 12px;
    padding: 16px;
    border: 2px solid var(--red);
  }
  .power.wide {
    gap: 14px;
    padding: 20px;
  }
  .intro {
    display: flex;
    flex-direction: column;
    gap: 6px;
  }
  .red {
    color: var(--red);
  }
  .text {
    font-size: 13px;
  }
  .wide .text {
    font-size: 14px;
  }
  .buttons {
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 10px;
  }
  .wide .buttons {
    gap: 12px;
  }
</style>
