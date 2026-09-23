<script lang="ts">
  import Appearance from "./components/Appearance.svelte";
  import CameraView from "./components/CameraView.svelte";
  import ConnectionBanner from "./components/ConnectionBanner.svelte";
  import ControlTiles from "./components/ControlTiles.svelte";
  import Diagnostics from "./components/Diagnostics.svelte";
  import Header from "./components/Header.svelte";
  import HostInfo from "./components/HostInfo.svelte";
  import Notice from "./components/Notice.svelte";
  import PhoneAccess from "./components/PhoneAccess.svelte";
  import ProfilePicker from "./components/ProfilePicker.svelte";
  import StatusList from "./components/StatusList.svelte";
  import SystemMenu from "./components/SystemMenu.svelte";
  import TopDown from "./components/TopDown.svelte";
  import ViewSwitch from "./components/ViewSwitch.svelte";
  import WarningStrip from "./components/WarningStrip.svelte";
  import { ui } from "./lib/ui.svelte";
</script>

{#if ui.wide}
  <div class="page wide">
    <Header wide />
    <ConnectionBanner />
    <Notice />

    {#if ui.tab === "main"}
      <div class="split">
        <aside class="controls"><ControlTiles /></aside>
        <main class="main">
          <div class="views">
            <CameraView />
            <TopDown />
          </div>
          <StatusList columns={2} />
          <div class="grow"></div>
          <WarningStrip />
        </main>
      </div>
    {:else if ui.tab === "diagnostics"}
      <Diagnostics wide />
    {:else}
      <div class="split">
        <aside class="controls"><ProfilePicker wide /></aside>
        <main class="main system">
          <HostInfo wide />
          <PhoneAccess />
          <Appearance wide />
          <div class="grow"></div>
          <SystemMenu wide />
        </main>
      </div>
    {/if}
  </div>
{:else}
  <div class="page narrow">
    <Header wide={false} />
    <ConnectionBanner />
    <Notice />

    {#if ui.tab === "main"}
      <ViewSwitch />
      <div class="rule"><ControlTiles compact /></div>
      <StatusList />
      <div class="pad"><WarningStrip compact /></div>
    {:else if ui.tab === "diagnostics"}
      <Diagnostics wide={false} />
    {:else}
      <div class="system-narrow">
        <HostInfo wide={false} />
        <ProfilePicker wide={false} />
        <Appearance wide={false} />
        <SystemMenu wide={false} />
      </div>
    {/if}
  </div>
{/if}

<style>
  .page {
    display: flex;
    flex-direction: column;
    background: var(--bg);
    color: var(--ink);
  }
  .wide {
    height: 100dvh;
    overflow: hidden;
  }
  .narrow {
    min-height: 100dvh;
    padding-bottom: env(safe-area-inset-bottom);
  }
  .split {
    flex-grow: 1;
    display: flex;
    min-height: 0;
  }
  .controls {
    width: 380px;
    flex-shrink: 0;
    display: flex;
    flex-direction: column;
    border-right: 2px solid var(--ink);
    overflow-y: auto;
  }
  .main {
    flex-grow: 1;
    min-width: 0;
    display: flex;
    flex-direction: column;
    gap: 24px;
    padding: 26px 28px;
    overflow-y: auto;
  }
  .main.system {
    gap: 28px;
  }
  .views {
    display: grid;
    grid-template-columns: minmax(0, 16fr) minmax(0, 9fr);
    gap: 16px;
    align-items: start;
  }
  .grow {
    flex-grow: 1;
  }
  .rule {
    border-top: 2px solid var(--ink);
  }
  .pad {
    padding: 0 16px 16px;
  }
  .system-narrow {
    display: flex;
    flex-direction: column;
    gap: 24px;
    padding: 4px 16px 20px;
  }
</style>
