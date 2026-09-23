<script lang="ts">
  import { ui } from "../lib/ui.svelte";

  let { wide }: { wide: boolean } = $props();
</script>

<div class="appearance">
  <span class="intro">
    <span class="label">APPEARANCE</span>
    {#if wide}<span class="muted note">Saved on this device</span>{/if}
  </span>
  <div class="seg" role="group" aria-label="Appearance">
    <button
      style:width="{wide ? 96 : 80}px"
      aria-pressed={!ui.dark}
      onclick={() => ui.setDark(false)}>LIGHT</button
    >
    <button style:width="{wide ? 96 : 80}px" aria-pressed={ui.dark} onclick={() => ui.setDark(true)}
      >DARK</button
    >
  </div>
</div>

<style>
  .appearance {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 12px;
  }
  .intro {
    display: flex;
    flex-direction: column;
    gap: 6px;
  }
  .note {
    font-size: 14px;
  }
</style>
