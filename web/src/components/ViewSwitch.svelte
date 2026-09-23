<script lang="ts">
  // Narrow layout: one full-width view at a time, camera or top-down.
  import { ui } from "../lib/ui.svelte";
  import CameraView from "./CameraView.svelte";
  import TopDown from "./TopDown.svelte";
</script>

<div class="viewswitch">
  <div class="seg" role="group" aria-label="View">
    <button aria-pressed={ui.view === "camera"} onclick={() => (ui.view = "camera")}>CAMERA</button>
    <button aria-pressed={ui.view === "topdown"} onclick={() => (ui.view = "topdown")}
      >TOP DOWN</button
    >
  </div>
  {#if ui.view === "camera"}
    <CameraView showLabel={false} />
  {:else}
    <TopDown showLabel={false} />
  {/if}
</div>

<style>
  .viewswitch {
    display: flex;
    flex-direction: column;
    gap: 8px;
    padding: 0 16px 12px;
  }
  .seg {
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
  }
  .seg > button {
    font-size: 12px;
    letter-spacing: 0.1em;
  }
</style>
