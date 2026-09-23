<script lang="ts">
  // Wide layout only: the Wi-Fi access toggle, addresses, and a QR code for the phone.
  import qrcode from "qrcode-generator";
  import { status } from "../lib/status.svelte";
  import Toggle from "./Toggle.svelte";

  const net = $derived(status.network);
  const open = $derived(net?.wifi_access ?? false);
  const url = $derived(net?.wifi_address ? `http://${net.wifi_address}` : null);

  let busy = $state(false);
  async function set(enabled: boolean) {
    if (busy) return;
    busy = true;
    try {
      await status.command("/command/set_wifi_access", { enabled });
    } finally {
      busy = false;
    }
  }

  const qr = $derived.by(() => {
    if (!url) return null;
    const code = qrcode(0, "M");
    code.addData(url);
    code.make();
    const n = code.getModuleCount();
    let d = "";
    for (let r = 0; r < n; r++) {
      for (let c = 0; c < n; c++) if (code.isDark(r, c)) d += `M${c} ${r}h1v1h-1z`;
    }
    return { n, d };
  });

  const wifiRow = $derived.by(() => {
    if (!net) return "--";
    if (!net.wifi_address) return `no address · ${net.wifi_interface || "no Wi-Fi"}`;
    return open ? `${net.wifi_address} · ${net.wifi_interface}` : "closed";
  });
</script>

<div class="phone">
  <div class="left">
    <span class="label section-head">PHONE ACCESS</span>
    <div class="toggle-row">
      <span class="text">
        <span class="title">Allow on Wi-Fi</span>
        <span class="muted note"
          >{open
            ? "Open on Wi-Fi until you turn it off, including after a reboot."
            : "Cable only. The box refuses dashboard connections over Wi-Fi."}</span
        >
      </span>
      <Toggle
        label="Allow on Wi-Fi"
        on={open}
        disabled={!net || busy || !status.appUp}
        onchange={set}
      />
    </div>
    <div class="leader">
      <span class="k">Wi-Fi address</span><span class="dots"></span><span class="v">{wifiRow}</span>
    </div>
    <div class="leader">
      <span class="k">Phone URL</span><span class="dots"></span><span class="v"
        >{net ? `${net.hostname}.local` : "--"}</span
      >
    </div>
  </div>
  <div class="qr-wrap">
    <div class="qr">
      {#if qr && open}
        <svg
          viewBox="-2 -2 {qr.n + 4} {qr.n + 4}"
          role="img"
          aria-label="QR code for {url}"
          shape-rendering="crispEdges"
        >
          <path d={qr.d} />
        </svg>
      {:else}
        <span class="mono noqr">{open ? "NO WI-FI ADDRESS" : "WI-FI CLOSED"}</span>
      {/if}
    </div>
    <span class="label">SCAN ON PHONE</span>
  </div>
</div>

<style>
  .phone {
    display: flex;
    gap: 20px;
    align-items: flex-end;
  }
  .left {
    flex-grow: 1;
    min-width: 0;
    display: flex;
    flex-direction: column;
  }
  .toggle-row {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 16px;
    height: 64px;
    border-bottom: 1px solid var(--ink);
  }
  .text {
    display: flex;
    flex-direction: column;
    gap: 5px;
    min-width: 0;
  }
  .title {
    font-size: 14px;
    font-weight: 600;
  }
  .note {
    font-size: 12px;
  }
  .qr-wrap {
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 6px;
    flex-shrink: 0;
  }
  .qr {
    width: 128px;
    height: 128px;
    border: 2px solid var(--ink);
    background: #ffffff;
    display: flex;
    align-items: center;
    justify-content: center;
  }
  .qr svg {
    display: block;
    width: 124px;
    height: 124px;
  }
  .qr path {
    fill: #121212;
  }
  .noqr {
    padding: 8px;
    text-align: center;
    font-size: 10px;
    color: #8a8a85;
  }
</style>
