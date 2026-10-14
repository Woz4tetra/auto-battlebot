<script lang="ts">
  // Control row 05: the running camera profile. Folded it is one row like the others; CHANGE
  // unfolds the profile list, and picking one applies it and folds the row back up. Editing
  // lives on the CAMERA tab.

  import { applyProfile, summary } from "../lib/camera";
  import { status } from "../lib/status.svelte";

  let { compact = false }: { compact?: boolean } = $props();

  const cam = $derived(status.camera);
  const live = $derived(status.appUp && !!cam?.supported);

  let open = $state(false);
  let busy = $state(false);
  let appliedAt = $state<string | null>(null);

  const sub = $derived.by(() => {
    if (!cam) return "--";
    if (!cam.supported) return "NO LIVE CONTROLS";
    if (appliedAt && !open) return `APPLIED ${appliedAt}`;
    return (cam.modified ? "EDITED · " : "") + summary(cam.settings);
  });

  async function pick(name: string) {
    if (busy) return;
    busy = true;
    try {
      const ack = await applyProfile(name);
      if (ack?.accepted) {
        appliedAt = new Date().toLocaleTimeString([], { hour12: false });
        open = false;
      }
    } finally {
      busy = false;
    }
  }

  function toggle() {
    open = !open;
    appliedAt = null;
  }
</script>

<section class="tile" class:compact class:open aria-label="Camera profile">
  <div class="row">
    <span class="rownum">05</span>
    <span class="body">
      <span class="label">CAMERA</span>
      <span class="state name">{cam?.supported ? cam.active_profile : "--"}</span>
      <span class="sub mono" class:on={appliedAt !== null && !open}>
        <span class="subdot"></span>{sub}
      </span>
    </span>
    <button class="btn" aria-expanded={open} class:pressed={open} disabled={!live} onclick={toggle}
      >{open ? "CLOSE" : "CHANGE"}</button
    >
  </div>
  {#if open && cam}
    <div class="list">
      {#each cam.profiles as name (name)}
        <button
          class="profile mono"
          aria-pressed={name === cam.active_profile}
          disabled={busy || !live}
          onclick={() => pick(name)}
        >
          <span>{name}</span>
          {#if name === cam.active_profile}<span class="label">LIVE</span>{/if}
        </button>
      {/each}
    </div>
  {/if}
</section>

<style>
  .tile {
    border-bottom: 1px solid var(--ink);
  }
  .tile.open {
    background: var(--hazard);
  }
  .row {
    display: flex;
    align-items: center;
    gap: 16px;
    height: 92px;
    padding: 0 28px;
  }
  .compact .row {
    gap: 12px;
    height: 80px;
    padding: 0 16px;
  }
  .rownum {
    width: 20px;
    flex-shrink: 0;
  }
  .compact .rownum {
    width: 18px;
  }
  .body {
    flex-grow: 1;
    min-width: 0;
    display: flex;
    flex-direction: column;
    gap: 8px;
  }
  .name {
    font-size: 20px;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }
  .sub {
    display: flex;
    align-items: center;
    gap: 6px;
    font-size: 12px;
    font-weight: 600;
    letter-spacing: 0.04em;
    color: var(--muted);
    white-space: nowrap;
    overflow: hidden;
  }
  .subdot {
    width: 8px;
    height: 8px;
    flex-shrink: 0;
    border: 1.5px solid var(--muted);
    border-radius: 50%;
  }
  .sub.on {
    color: var(--green);
  }
  .sub.on .subdot {
    background: var(--green);
    border-color: var(--green);
  }
  .btn {
    flex-shrink: 0;
  }
  .btn.pressed {
    background: var(--ink);
    color: var(--bg);
  }
  .list {
    display: flex;
    flex-direction: column;
    gap: 6px;
    padding: 0 28px 16px 64px;
  }
  .compact .list {
    padding: 0 16px 14px 46px;
  }
  .profile {
    display: flex;
    align-items: center;
    justify-content: space-between;
    height: 44px;
    padding: 0 14px;
    border: 2px solid var(--ink);
    background: transparent;
    color: var(--ink);
    font-size: 14px;
    font-weight: 600;
    text-align: left;
  }
  .profile[aria-pressed="true"] {
    background: var(--ink);
    color: var(--bg);
  }
  .profile:disabled {
    color: var(--faint);
    border-color: var(--faint);
  }
  .profile[aria-pressed="true"]:disabled {
    background: var(--faint);
    color: var(--bg);
  }
</style>
