// Reactive state for every status topic, the connection, and the relay's /healthz.

import { connection, type CommandAck, type ConnectionState } from "./connection";
import type {
  AppStatus,
  CommandPayloads,
  CommandTopic,
  NetworkStatus,
  SticksStatus,
  SystemStatus,
  TracksStatus,
} from "../generated/protocol";

/** The app counts as down after this long without /status/system. */
export const APP_TIMEOUT_MS = 2000;
const HEALTH_PERIOD_MS = 5000;

export type LinkKind = "cable" | "wifi" | "local";

interface Health {
  app_connected: boolean;
  link: LinkKind;
}

class StatusStore {
  connection = $state<ConnectionState>("closed");
  system = $state.raw<SystemStatus | null>(null);
  app = $state.raw<AppStatus | null>(null);
  sticks = $state.raw<SticksStatus | null>(null);
  tracks = $state.raw<TracksStatus | null>(null);
  network = $state.raw<NetworkStatus | null>(null);
  health = $state.raw<Health | null>(null);
  /** Last ack that carried a message, shown as a notice. */
  notice = $state.raw<{ text: string; ok: boolean; at: number } | null>(null);

  systemAt = $state(0);
  /** Wall clock in ms, ticking twice a second, for "time since" displays. */
  now = $state(Date.now());
  /** When recording was first seen on, for the header timer. */
  recordingSince = $state<number | null>(null);

  appUp = $derived(this.connection === "open" && this.now - this.systemAt < APP_TIMEOUT_MS);
  recording = $derived(!!this.system && (this.system.svo_recording || this.system.mcap_recording));

  #started = false;

  start(): void {
    if (this.#started) return;
    this.#started = true;

    connection.onState((s) => {
      this.connection = s;
    });
    connection.subscribe("/status/system", (m) => {
      const recording = m.svo_recording || m.mcap_recording;
      if (recording && this.recordingSince === null) this.recordingSince = Date.now();
      if (!recording) this.recordingSince = null;
      this.system = m;
      this.systemAt = Date.now();
    });
    connection.subscribe("/status/app", (m) => (this.app = m));
    connection.subscribe("/status/sticks", (m) => (this.sticks = m));
    connection.subscribe("/status/tracks", (m) => (this.tracks = m));
    connection.subscribe("/status/network", (m) => (this.network = m));
    connection.start();

    setInterval(() => (this.now = Date.now()), 500);
    void this.#pollHealth();
    setInterval(() => void this.#pollHealth(), HEALTH_PERIOD_MS);
  }

  /** Sends a command and records the ack message (or the failure) as the notice. */
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
