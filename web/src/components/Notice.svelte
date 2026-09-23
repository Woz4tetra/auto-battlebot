<script lang="ts">
  // Shows the message of the latest command ack, or why a command failed, for a few seconds.
  import { status } from "../lib/status.svelte";

  const VISIBLE_MS = 6000;
  const visible = $derived(status.notice !== null && status.now - status.notice.at < VISIBLE_MS);
</script>

{#if visible && status.notice}
  <div class="notice" class:bad={!status.notice.ok} role="status">
    <span class="tag">{status.notice.ok ? "ACK" : "FAILED"}</span>
    <span class="text">{status.notice.text}</span>
    <button class="close" aria-label="Dismiss" onclick={() => (status.notice = null)}>×</button>
  </div>
{/if}

<style>
  .notice {
    display: flex;
    align-items: center;
    gap: 12px;
    min-height: 44px;
    padding: 0 0 0 16px;
    border-bottom: 2px solid var(--ink);
    font-size: 14px;
    font-weight: 600;
  }
  .tag {
    flex-shrink: 0;
    padding: 3px 7px;
    background: var(--ink);
    color: var(--bg);
    font-family: var(--mono);
    font-size: 11px;
    letter-spacing: 0.06em;
  }
  .bad .tag {
    background: var(--red);
    color: #ffffff;
  }
  .text {
    flex-grow: 1;
    min-width: 0;
    overflow-wrap: anywhere;
  }
  .close {
    width: 44px;
    height: 44px;
    border: none;
    background: transparent;
    font-size: 22px;
    line-height: 1;
  }
</style>
