<script lang="ts">
  import { ui } from "../lib/ui.svelte";

  let { size = 40 }: { size?: number } = $props();
</script>

<button
  aria-label="Dark mode"
  aria-pressed={ui.dark}
  style:width="{size}px"
  style:height="{size}px"
  onclick={() => ui.setDark(!ui.dark)}
>
  {#if ui.dark}
    <svg
      width="20"
      height="20"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      stroke-width="2"
      stroke-linecap="square"
    >
      <circle cx="12" cy="12" r="4.5" />
      <path
        d="M12 1.5v3M12 19.5v3M1.5 12h3M19.5 12h3M4.6 4.6l2.1 2.1M17.3 17.3l2.1 2.1M4.6 19.4l2.1-2.1M17.3 6.7l2.1-2.1"
      />
    </svg>
  {:else}
    <svg
      width="20"
      height="20"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      stroke-width="2"
      stroke-linejoin="miter"
    >
      <path d="M20.5 14.5A8.5 8.5 0 1 1 9.5 3.5a7 7 0 0 0 11 11z" />
    </svg>
  {/if}
</button>

<style>
  button {
    flex-shrink: 0;
    display: flex;
    align-items: center;
    justify-content: center;
    padding: 0;
    border: 2px solid var(--ink);
    background: transparent;
    color: var(--ink);
  }
</style>
