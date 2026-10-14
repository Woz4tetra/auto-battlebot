<script lang="ts">
  // CAMERA tab controls: profile list, live sliders, and save/delete. Every slider move goes to
  // the camera at once; nothing is written to disk until SAVE or SAVE AS.

  import type { CameraSettings } from "../generated/protocol";
  import {
    ANTI_BANDING,
    CONTROLS,
    DEFAULT_PROFILE,
    applyProfile,
    blurMm,
    summary,
  } from "../lib/camera";
  import { status } from "../lib/status.svelte";
  import HoldButton from "./HoldButton.svelte";
  import Toggle from "./Toggle.svelte";

  let { wide }: { wide: boolean } = $props();

  const SEND_INTERVAL_MS = 80;

  const cam = $derived(status.camera);
  const live = $derived(status.appUp && !!cam?.supported);
  const onDefault = $derived(cam?.active_profile === DEFAULT_PROFILE);

  /** Local copy while a slider is being dragged, so read-backs do not fight the thumb. */
  let draft = $state<CameraSettings | null>(null);
  let releaseTimer = 0;
  const shown = $derived(draft ?? cam?.settings ?? null);

  let newName = $state("");
  let busy = $state(false);

  let lastSent = 0;
  let pending: CameraSettings | null = null;
  let sendTimer = 0;

  function send(next: CameraSettings) {
    pending = next;
    const wait = lastSent + SEND_INTERVAL_MS - Date.now();
    if (wait > 0) {
      if (!sendTimer) sendTimer = window.setTimeout(flush, wait);
      return;
    }
    flush();
  }

  function flush() {
    sendTimer = 0;
    if (!pending) return;
    const settings = pending;
    pending = null;
    lastSent = Date.now();
    void status.command("/command/set_camera_settings", { settings });
  }

  function change(patch: Partial<CameraSettings>) {
    if (!shown) return;
    const next = { ...shown, ...patch };
    draft = next;
    window.clearTimeout(releaseTimer);
    // Hand the display back to the read-back once the camera has had time to settle.
    releaseTimer = window.setTimeout(() => (draft = null), 1500);
    send(next);
  }

  function slider(key: (typeof CONTROLS)[number]["key"], e: Event) {
    change({ [key]: Number((e.currentTarget as HTMLInputElement).value) });
  }

  async function run(fn: () => Promise<unknown>) {
    if (busy) return;
    busy = true;
    try {
      await fn();
    } finally {
      busy = false;
    }
  }

  const validName = $derived(
    /^[A-Za-z0-9_-]{1,32}$/.test(newName) && newName.toUpperCase() !== DEFAULT_PROFILE,
  );

  function saveAs() {
    const name = newName;
    void run(async () => {
      const ack = await status.command("/command/camera_profile", { action: "save", name });
      if (ack?.accepted) newName = "";
    });
  }

  const note = $derived.by(() => {
    if (!cam) return "Waiting for app.";
    if (!cam.supported) return "This camera has no live controls.";
    if (cam.modified) {
      return onDefault
        ? "Live edits on DEFAULT. Save them as a new profile to keep them."
        : `Live edits not saved to ${cam.active_profile}.`;
    }
    return "Changes apply on the next frame. The app always starts on DEFAULT.";
  });

  $effect(() => () => {
    window.clearTimeout(releaseTimer);
    window.clearTimeout(sendTimer);
  });
</script>

<div class="settings" class:wide>
  <div class="head" class:section-head={!wide}>
    <span class="label">CAMERA PROFILES</span>
  </div>
  {#each cam?.profiles ?? [] as name (name)}
    <button
      class="profile"
      aria-pressed={cam?.active_profile === name}
      disabled={!live || busy}
      onclick={() => run(() => applyProfile(name))}
    >
      <span class="box"></span>
      <span class="name mono">{name}</span>
      {#if cam?.active_profile === name}
        <span class="label tag" class:edited={cam.modified}>{cam.modified ? "EDITED" : "LIVE"}</span
        >
      {/if}
    </button>
  {:else}
    <div class="empty muted">{status.appUp ? "No camera." : "Waiting for app."}</div>
  {/each}

  {#if cam?.supported && shown}
    <div class="group-head">
      <span class="label">EXPOSURE</span>
      <span class="grow"></span>
      <span class="label muted">AUTO</span>
      <Toggle
        label="Auto exposure"
        on={shown.auto_exposure}
        disabled={!live}
        onchange={(on) => change({ auto_exposure: on })}
      />
    </div>
    {#each CONTROLS as c (c.key)}
      {@const range = cam.limits[c.key]}
      {#if c.key === "white_balance_k"}
        <div class="group-head">
          <span class="label">COLOR</span>
          <span class="grow"></span>
          <span class="label muted">AUTO WB</span>
          <Toggle
            label="Auto white balance"
            on={shown.auto_white_balance}
            disabled={!live}
            onchange={(on) => change({ auto_white_balance: on })}
          />
        </div>
      {/if}
      {#if range.supported}
        {@const locked = c.auto !== undefined && shown[c.auto]}
        <label class="control" class:locked>
          <span class="control-head">
            <span class="label">{c.label}</span>
            <span class="grow"></span>
            <span class="mono value">{c.format(shown[c.key])}</span>
          </span>
          <input
            type="range"
            min={range.min}
            max={range.max}
            step={c.step}
            value={shown[c.key]}
            disabled={!live || locked}
            oninput={(e) => slider(c.key, e)}
          />
        </label>
      {/if}
    {/each}
    {#if cam.limits.anti_banding.supported}
      <div class="control banding">
        <span class="label">ANTI-BANDING</span>
        <span class="grow"></span>
        <div class="seg" role="group" aria-label="Anti-banding">
          {#each ANTI_BANDING as b (b.value)}
            <button
              class="mono"
              aria-pressed={shown.anti_banding === b.value}
              disabled={!live}
              onclick={() => change({ anti_banding: b.value })}>{b.label}</button
            >
          {/each}
        </div>
      </div>
    {/if}
    <div class="readout mono muted">
      {summary(shown)} · BLUR AT 3 M/S {blurMm(shown.exposure_us)} MM
    </div>
  {/if}

  {#if wide}<div class="grow-v"></div>{/if}

  <div class="actions">
    <span class="note muted">{note}</span>
    <div class="buttons">
      <button
        class="btn"
        disabled={!live || busy || !cam?.modified}
        onclick={() => run(() => applyProfile(cam?.active_profile ?? DEFAULT_PROFILE))}
        >REVERT</button
      >
      <button
        class="btn"
        class:filled={cam?.modified && !onDefault}
        disabled={!live || busy || onDefault || !cam?.modified}
        onclick={() =>
          run(() =>
            status.command("/command/camera_profile", {
              action: "save",
              name: cam?.active_profile ?? "",
            }),
          )}>SAVE</button
      >
    </div>
    <div class="save-as">
      <input
        class="mono"
        type="text"
        placeholder="new_profile_name"
        maxlength="32"
        autocomplete="off"
        spellcheck="false"
        aria-label="New profile name"
        bind:value={newName}
        disabled={!live}
      />
      <button class="btn" disabled={!live || busy || !validName} onclick={saveAs}>SAVE AS</button>
    </div>
    {#if !onDefault && cam?.supported}
      <HoldButton
        height={44}
        danger
        disabled={!live || busy}
        onconfirm={() =>
          run(() =>
            status.command("/command/camera_profile", {
              action: "delete",
              name: cam?.active_profile ?? "",
            }),
          )}>HOLD TO DELETE {cam?.active_profile}</HoldButton
      >
    {/if}
  </div>
</div>

<style>
  .settings {
    display: flex;
    flex-direction: column;
  }
  .settings.wide {
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
    font-size: 14px;
    text-align: left;
  }
  .wide .profile {
    padding: 0 28px;
  }
  .box {
    width: 16px;
    height: 16px;
    flex-shrink: 0;
    border: 2px solid var(--ink);
  }
  .profile[aria-pressed="true"] {
    background: var(--hazard);
  }
  .profile[aria-pressed="true"] .box {
    background: var(--ink);
  }
  .name {
    flex-grow: 1;
    font-weight: 600;
  }
  .tag {
    color: var(--green);
  }
  .tag.edited {
    color: var(--amber);
  }
  .empty {
    padding: 16px 12px;
  }
  .group-head {
    display: flex;
    align-items: center;
    gap: 12px;
    height: 60px;
    flex-shrink: 0;
    padding: 0 12px;
    border-bottom: 2px solid var(--ink);
    margin-top: 12px;
  }
  .wide .group-head,
  .wide .control,
  .wide .readout {
    padding-left: 28px;
    padding-right: 28px;
  }
  .control {
    display: flex;
    flex-direction: column;
    gap: 2px;
    flex-shrink: 0;
    padding: 8px 12px 2px;
    border-bottom: 1px solid var(--grid);
  }
  .control.locked {
    opacity: 0.5;
  }
  .control-head {
    display: flex;
    align-items: baseline;
  }
  .value {
    font-size: 14px;
    font-weight: 600;
  }
  .banding {
    flex-direction: row;
    align-items: center;
    padding-top: 10px;
    padding-bottom: 10px;
  }
  .banding .seg > button {
    min-width: 50px;
    font-size: 13px;
  }
  input[type="range"] {
    width: 100%;
    height: 32px;
    margin: 0;
    accent-color: var(--ink);
  }
  .readout {
    padding: 10px 12px;
    font-size: 12px;
  }
  .grow {
    flex-grow: 1;
  }
  .grow-v {
    flex-grow: 1;
  }
  .actions {
    display: flex;
    flex-direction: column;
    gap: 10px;
    padding: 16px 12px;
    border-top: 2px solid var(--ink);
  }
  .wide .actions {
    padding: 16px 28px 20px;
  }
  .note {
    font-size: 13px;
    line-height: 1.4;
  }
  .buttons {
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 8px;
  }
  .btn.filled {
    background: var(--ink);
    color: var(--bg);
  }
  .save-as {
    display: flex;
    gap: 8px;
  }
  .save-as input {
    flex-grow: 1;
    min-width: 0;
    height: 44px;
    padding: 0 12px;
    border: 2px solid var(--ink);
    background: transparent;
    color: var(--ink);
    font-size: 14px;
  }
  .save-as input:focus-visible {
    outline: 3px solid var(--amber);
    outline-offset: 2px;
  }
</style>
