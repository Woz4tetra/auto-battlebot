<script lang="ts">
  import { status } from "../lib/status.svelte";
  import DarkToggle from "./DarkToggle.svelte";
  import LinkPill from "./LinkPill.svelte";
  import Logo from "./Logo.svelte";
  import StatusBar from "./StatusBar.svelte";
  import Tabs from "./Tabs.svelte";

  let { wide }: { wide: boolean } = $props();

  const wifiOpen = $derived(status.network?.wifi_access === true);
</script>

{#if wide}
  <header class="wide">
    <Logo width={50} />
    <div class="title">
      <div class="name">AUTO-BATTLEBOT</div>
      <div class="profile mono">{status.app?.current_profile ?? "no profile"}</div>
    </div>
    <Tabs wide />
    <div class="grow"></div>
    <StatusBar />
    {#if wifiOpen}
      <span class="wifi" title="Dashboard is open on Wi-Fi">WI-FI</span>
    {/if}
    <LinkPill />
    <DarkToggle size={44} />
  </header>
{:else}
  <header class="narrow">
    <Logo width={40} />
    <span class="name" class:optional={wifiOpen}>AUTO-BATTLEBOT</span>
    {#if wifiOpen}
      <span class="wifi small" title="Dashboard is open on Wi-Fi">WI-FI</span>
    {/if}
    <LinkPill small />
    <DarkToggle size={44} />
  </header>
  <StatusBar strip />
  <Tabs wide={false} />
{/if}

<style>
  header {
    display: flex;
    align-items: center;
    flex-shrink: 0;
    border-bottom: 2px solid var(--ink);
  }
  .wide {
    height: 72px;
    padding: 0 28px;
    gap: 12px;
  }
  .wide > :global(*) {
    flex-shrink: 0;
  }
  .narrow {
    height: 60px;
    padding: 0 16px;
    gap: 10px;
  }
  .wide > .title {
    display: flex;
    flex-direction: column;
    gap: 5px;
    flex-shrink: 1;
    min-width: 0;
    overflow: hidden;
  }
  .name {
    font-weight: 800;
    font-stretch: 125%;
    font-size: 21px;
    letter-spacing: 0.02em;
    line-height: 1;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }
  .narrow .name {
    flex-grow: 1;
    min-width: 0;
    font-size: 16px;
    font-stretch: 116%;
    letter-spacing: 0;
    overflow: hidden;
    text-overflow: ellipsis;
  }
  /* On a phone the amber WI-FI tag takes the title's room; the logo stays. */
  @media (max-width: 439px) {
    .narrow .name.optional {
      visibility: hidden;
    }
  }
  .profile {
    font-size: 12px;
    color: var(--muted);
    line-height: 1;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }
  .grow {
    flex-grow: 1;
  }
  /* iPad landscape (1180 px) fits the title at 18 px; below that it makes way entirely. */
  @media (max-width: 1279px) {
    .wide .name {
      font-size: 18px;
      font-stretch: 112%;
      letter-spacing: 0;
    }
  }
  @media (max-width: 1139px) {
    .wide .title {
      display: none;
    }
  }
  .wifi {
    display: flex;
    align-items: center;
    flex-shrink: 0;
    height: 36px;
    padding: 0 10px;
    border: 2px solid var(--amber);
    color: var(--amber);
    font-family: var(--mono);
    font-size: 12px;
    font-weight: 600;
    letter-spacing: 0.08em;
  }
  .wifi.small {
    height: 30px;
    padding: 0 6px;
    font-size: 11px;
  }
</style>
