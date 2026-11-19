import { connection, type CommandAck, type ConnectionState } from "./connection";
import type {
  AppStatus,
  BatteryStatus,
  CameraSettingsStatus,
  CommandPayloads,
  CommandTopic,
  NetworkStatus,
  ResolvedConfigStatus,
  SticksStatus,
  SystemStatus,
  TracksStatus,
} from "../generated/protocol";

export const APP_DOWN_AFTER_MS = 2000;
const HEALTH_PERIOD_MS = 5000;
const NEW_FILE_JUMP_MS = 1000;

export type LinkKind = "cable" | "wifi" | "local";

interface Health {
  app_connected: boolean;
  link: LinkKind;
}

class StatusStore {
  connection = $state<ConnectionState>("closed");
  system = $state.raw<SystemStatus | null>(null);
  app = $state.raw<AppStatus | null>(null);
  config = $state.raw<ResolvedConfigStatus | null>(null);
  sticks = $state.raw<SticksStatus | null>(null);
  tracks = $state.raw<TracksStatus | null>(null);
  network = $state.raw<NetworkStatus | null>(null);
  battery = $state.raw<BatteryStatus | null>(null);
  camera = $state.raw<CameraSettingsStatus | null>(null);
  health = $state.raw<Health | null>(null);
  notice = $state.raw<{ text: string; ok: boolean; at: number } | null>(null);

  systemAt = $state(0);
  nowMs = $state(Date.now());
  recordingSince = $state<number | null>(null);
  mcapFileSince = $state<number | null>(null);

  appUp = $derived(this.connection === "open" && this.nowMs - this.systemAt < APP_DOWN_AFTER_MS);
  recording = $derived(
    !!this.system && (this.system.camera_recording || this.system.mcap_recording),
  );

  #started = false;

  start(): void {
    if (this.#started) return;
    this.#started = true;

    connection.onState((s) => {
      this.connection = s;
    });
    connection.subscribe("/status/system", (m) => {
      const recording = m.camera_recording || m.mcap_recording;
      if (recording && this.recordingSince === null) this.recordingSince = Date.now();
      if (!recording) this.recordingSince = null;
      const since = m.mcap_file_s == null ? null : Date.now() - m.mcap_file_s * 1000;
      if (
        since === null ||
        this.mcapFileSince === null ||
        Math.abs(since - this.mcapFileSince) > NEW_FILE_JUMP_MS
      ) {
        this.mcapFileSince = since;
      }
      this.system = m;
      this.systemAt = Date.now();
    });
    connection.subscribe("/status/app", (m) => (this.app = m));
    connection.subscribe("/status/config", (m) => (this.config = m));
    connection.subscribe("/status/sticks", (m) => (this.sticks = m));
    connection.subscribe("/status/tracks", (m) => (this.tracks = m));
    connection.subscribe("/status/network", (m) => (this.network = m));
    connection.subscribe("/status/battery", (m) => (this.battery = m));
    connection.subscribe("/status/camera_settings", (m) => (this.camera = m));
    connection.start();

    setInterval(() => (this.nowMs = Date.now()), 500);
    void this.#pollHealth();
    setInterval(() => void this.#pollHealth(), HEALTH_PERIOD_MS);
  }

  async command<T extends CommandTopic>(
    topic: T,
    payload: CommandPayloads[T],
  ): Promise<CommandAck | null> {
    try {
      const ack = await connection.send(topic, payload);
      if (ack.message || !ack.accepted) {
        this.notice = {
          text: ack.message || `${topic} rejected`,
          ok: ack.accepted,
          at: Date.now(),
        };
      }
      return ack;
    } catch (err) {
      const reason = err instanceof Error ? err.message : String(err);
      this.notice = {
        text: `${topic.replace("/command/", "")}: ${reason}`,
        ok: false,
        at: Date.now(),
      };
      return null;
    }
  }

  async #pollHealth(): Promise<void> {
    try {
      const res = await fetch("/healthz", { cache: "no-store" });
      if (!res.ok) throw new Error(String(res.status));
      const body = (await res.json()) as Partial<Health>;
      this.health = {
        app_connected: !!body.app_connected,
        link: body.link === "cable" || body.link === "wifi" ? body.link : "local",
      };
    } catch {
      this.health = null;
    }
  }
}

export const status = new StatusStore();
