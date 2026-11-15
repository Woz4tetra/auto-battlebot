import { connection } from "./connection";

export type Level = 0 | 1 | 2 | 3;
export const LEVEL_NAMES = ["OK", "WARN", "ERROR", "STALE"] as const;

type Value = number | string | null;

interface Subsection {
  level: number;
  message: string;
  age_ms?: number;
  /** Not logged since the last message: the values are repeated. */
  held?: boolean;
  values: Record<string, Value>;
}
type Payload = Record<string, Subsection>;

export interface DiagValue {
  key: string;
  value: Value;
  repeatedAgeMs: number | null;
}

export interface DiagModule {
  id: string;
  level: Level;
  message: string;
  updatedAt: number;
  values: DiagValue[];
}

const PREFIX = "/diagnostics/";
const HISTORY_MS = 60_000;
const MODULES_REBUILD_HZ = 4;

const SEVERITY_BY_LEVEL: Record<Level, number> = { 0: 0, 1: 2, 2: 3, 3: 1 };

export function severity(level: Level): number {
  return SEVERITY_BY_LEVEL[level];
}

function flatten(id: string, payload: Payload): Omit<DiagModule, "updatedAt"> {
  let level: Level = 0;
  let message = "";
  const values: DiagValue[] = [];
  for (const [name, sub] of Object.entries(payload)) {
    if (!sub || typeof sub !== "object") continue;
    const subLevel = Math.max(0, Math.min(3, sub.level | 0)) as Level;
    if (severity(subLevel) > severity(level)) {
      level = subLevel;
      if (sub.message) message = sub.message;
    } else if (!message && sub.message) {
      message = sub.message;
    }
    const prefix = name === id ? "" : `${name}/`;
    const repeatedAgeMs = sub.held ? (sub.age_ms ?? 0) : null;
    for (const [key, value] of Object.entries(sub.values ?? {})) {
      values.push({ key: prefix + key, value, repeatedAgeMs });
    }
  }
  return { id, level, message, values };
}

class DiagnosticsStore {
  modules = $state.raw<DiagModule[]>([]);
  historyVersion = $state(0);

  #latest = new Map<string, DiagModule>();
  #history = new Map<string, Map<string, { t: number[]; v: number[] }>>();
  #unsubs = new Map<string, () => void>();
  #dirty = false;
  #started = false;

  start(): void {
    if (this.#started) return;
    this.#started = true;
    connection.onChannels((channels) => {
      for (const topic of channels.keys()) {
        if (topic.startsWith(PREFIX) && !this.#unsubs.has(topic)) {
          const id = topic.slice(PREFIX.length);
          this.#unsubs.set(
            topic,
            connection.subscribeJson<Payload>(topic, (payload) => this.#onMessage(id, payload)),
          );
        }
      }
    });
    setInterval(() => this.#flush(), 1000 / MODULES_REBUILD_HZ);
  }

  get sorted(): DiagModule[] {
    return [...this.modules].sort(
      (a, b) => severity(b.level) - severity(a.level) || a.id.localeCompare(b.id),
    );
  }

  get(id: string): DiagModule | undefined {
    return this.modules.find((m) => m.id === id);
  }

  history(id: string, key: string): { t: number[]; v: number[] } | undefined {
    return this.#history.get(id)?.get(key);
  }

  #onMessage(id: string, payload: Payload): void {
    const now = Date.now();
    const module = { ...flatten(id, payload), updatedAt: now };
    this.#latest.set(id, module);
    let perKey = this.#history.get(id);
    if (!perKey) {
      perKey = new Map();
      this.#history.set(id, perKey);
    }
    for (const { key, value, repeatedAgeMs } of module.values) {
      if (repeatedAgeMs !== null) continue;
      if (typeof value !== "number" || !Number.isFinite(value)) continue;
      let series = perKey.get(key);
      if (!series) {
        series = { t: [], v: [] };
        perKey.set(key, series);
      }
      series.t.push(now);
      series.v.push(value);
    }
    this.#dirty = true;
  }

  #flush(): void {
    if (!this.#dirty) return;
    this.#dirty = false;
    const cutoff = Date.now() - HISTORY_MS;
    for (const perKey of this.#history.values()) {
      for (const series of perKey.values()) {
        let drop = 0;
        while (drop < series.t.length && series.t[drop] < cutoff) drop++;
        if (drop > 0) {
          series.t.splice(0, drop);
          series.v.splice(0, drop);
        }
      }
    }
    this.modules = [...this.#latest.values()];
    this.historyVersion += 1;
  }
}

export const diagnostics = new DiagnosticsStore();
