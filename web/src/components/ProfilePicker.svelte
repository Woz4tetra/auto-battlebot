<script lang="ts">
  // Pick a profile from /status/app, then hold APPLY AND REBOOT: select_profile, then reboot.
  import { status } from "../lib/status.svelte";
  import HoldButton from "./HoldButton.svelte";

  let { wide }: { wide: boolean } = $props();

  let picked = $state<string | null>(null);
  let busy = $state(false);

  const running = $derived(status.app?.current_profile ?? null);
  const profiles = $derived(status.app?.available_profiles ?? []);
  const choice = $derived(picked ?? running);
  const changed = $derived(choice !== null && choice !== running);
  const note = $derived(
    changed
      ? `Selected ${choice}. It loads on the next boot.`
      : "Pick a profile, then apply. The app reboots to load it.",
  );

  async function apply() {
    if (!changed || !choice || busy) return;
    busy = true;
    try {
      const ack = await status.command("/command/select_profile", { name: choice });
      if (ack?.accepted) await status.command("/command/system_action", { action: "reboot_host" });
    } finally {
      busy = false;
    }
  }
</script>

<div class="picker" class:wide>
  <div class="head" class:section-head={!wide}>
    <span class="label">PROFILE</span>
  </div>
  {#each profiles as name (name)}
    <button class="profile" aria-pressed={choice === name} onclick={() => (picked = name)}>
      <span class="box"></span>
      <span class="name mono">{name}</span>
      {#if name === running}<span class="label running">RUNNING</span>{/if}
    </button>
  {:else}
    <div class="empty muted">{status.appUp ? "No profiles listed." : "Waiting for the app."}</div>
  {/each}
  {#if wide}<div class="grow"></div>{/if}
  <div class="apply">
    <span class="note muted">{note}</span>
    <HoldButton
      height={48}
      filled={changed}
      disabled={!changed || busy || !status.appUp}
      onconfirm={apply}>APPLY AND REBOOT</HoldButton
    >
  </div>
</div>

<style>
  .picker {
    display: flex;
    flex-direction: column;
  }
  .picker.wide {
    height: 100%;
  }
  .wide .head {
    display: flex;
    align-items: center;
    height: 64px;
    flex-shrink: 0;
    padding: 0 28px;
    border-bottom: 2px solid var(--ink);
  }
  .profile {
    display: flex;
    align-items: center;
    gap: 12px;
    flex-shrink: 0;
    height: 52px;
    padding: 0 12px;
    border: none;
    border-bottom: 1px solid var(--ink);
    background: transparent;
    color: var(--ink);
    text-align: left;
  }
  .wide .profile {
    gap: 14px;
    height: 60px;
    padding: 0 28px;
  }
  .profile[aria-pressed="true"] {
    background: var(--ink);
    color: var(--bg);
  }
  .box {
    width: 14px;
    height: 14px;
    flex-shrink: 0;
    border: 2px solid currentColor;
  }
  .profile[aria-pressed="true"] .box {
    background: var(--bg);
  }
  .name {
    flex-grow: 1;
    min-width: 0;
    font-size: 13px;
    font-weight: 600;
    overflow-wrap: anywhere;
  }
  .wide .name {
    font-size: 14px;
  }
  .running {
    font-size: 11px;
    flex-shrink: 0;
  }
  .wide .running {
    font-size: 12px;
  }
  .empty {
    padding: 14px 12px;
    font-size: 14px;
  }
  .wide .empty {
    padding: 14px 28px;
  }
  .grow {
    flex-grow: 1;
  }
  .apply {
    display: flex;
    flex-direction: column;
    gap: 10px;
    padding: 10px 0 0;
  }
  .wide .apply {
    padding: 20px 28px 24px;
  }
  .note {
    font-size: 13px;
  }
  .wide .note {
    font-size: 14px;
  }
</style>
