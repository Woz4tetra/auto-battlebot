<script lang="ts">
  // "Label .... value" status rows. Wide layout splits them into two columns.
  import { status } from "../lib/status.svelte";
  import { formatDuration } from "../lib/time";

  let { columns = 1 }: { columns?: 1 | 2 } = $props();

  const NONE = "--";

  const rows = $derived.by(() => {
    const sys = status.appUp ? status.system : null;
    const tracks = status.appUp ? status.tracks : null;
    const app = status.app;

    const transmitter = !sys
      ? NONE
      : sys.transmitter_receiving
        ? "RECEIVING"
        : sys.transmitter_connected
          ? "CONNECTED"
          : "DISCONNECTED";

    let range = NONE;
    const ours = tracks?.robots.find((r) => r.ours && !r.stale);
    if (ours && tracks) {
      const d = tracks.robots
        .filter((r) => !r.ours && !r.stale)
        .map((r) => Math.hypot(r.x - ours.x, r.y - ours.y));
      if (d.length) range = `${Math.min(...d).toFixed(2)} m`;
    }

    const rec = !sys
      ? NONE
      : [sys.svo_recording && "SVO", sys.mcap_recording && "MCAP"].filter(Boolean).join(" + ") ||
        "OFF";

    // Time since the radio's autonomy switch went on; OFF while it is off, NONE without one.
    const radioAutonomy = !sys
      ? NONE
      : sys.autonomy_switch_on === undefined
        ? "NONE"
        : sys.autonomy_switch_on
          ? formatDuration(sys.autonomy_on_s)
          : "OFF";

    const left = [
      { label: "App uptime", value: sys ? formatDuration(sys.uptime_s) : NONE },
      { label: "Camera", value: sys ? (sys.camera_ok ? "OK" : "FAULT") : NONE },
      { label: "Transmitter", value: transmitter },
      { label: "Field", value: sys ? (sys.initialized ? "READY" : "NOT READY") : NONE },
      {
        label: "Our robot",
        value: tracks ? (tracks.our_robot_seen ? "TRACKED" : "LOST") : NONE,
      },
      {
        label: "Opponents seen",
        value: tracks && sys ? `${tracks.opponents_seen} / ${sys.selected_opponent_count}` : NONE,
      },
    ];
    const right = [
      { label: "Radio switch", value: radioAutonomy },
      {
        label: "Loop rate",
        value: sys
          ? `${sys.loop_rate_hz.toFixed(1)}${app ? ` / ${app.max_loop_rate_hz.toFixed(0)}` : ""} Hz`
          : NONE,
      },
      {
        label: "Jetson temp",
        value:
          sys?.jetson_temperature_c !== undefined
            ? `${Math.round(sys.jetson_temperature_c)} °C`
            : NONE,
      },
      { label: "Compute mode", value: sys?.compute_mode || NONE },
      { label: "Target range", value: range },
      { label: "Recording", value: rec },
    ];
    return { left, right };
  });
</script>

{#if columns === 2}
  <div class="cols">
    {#each [rows.left, rows.right] as col, i (i)}
      <div class="col">
        {#each col as row (row.label)}
          <div class="leader">
            <span class="k">{row.label}</span><span class="dots"></span><span class="v"
              >{row.value}</span
            >
          </div>
        {/each}
      </div>
    {/each}
  </div>
{:else}
  <div class="col single">
    {#each [...rows.left, ...rows.right] as row (row.label)}
      <div class="leader">
        <span class="k">{row.label}</span><span class="dots"></span><span class="v"
          >{row.value}</span
        >
      </div>
    {/each}
  </div>
{/if}

<style>
  .cols {
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 32px;
  }
  .col {
    display: flex;
    flex-direction: column;
    min-width: 0;
  }
  .single {
    padding: 12px 16px;
  }
</style>
