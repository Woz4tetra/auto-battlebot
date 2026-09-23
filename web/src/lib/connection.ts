// Foxglove WebSocket client for viz_relay. One socket per page, reconnecting forever.
//
// Components subscribe by topic. The connection keeps the wanted topics across reconnects and
// channel re-advertisements, so a subscriber never has to resubscribe itself.

import {
  FoxgloveClient,
  type Channel,
  type IWebSocket,
  type MessageData,
} from "@foxglove/ws-protocol";
import {
  COMMAND_SCHEMAS,
  type CommandAckStatus,
  type CommandPayloads,
  type CommandTopic,
  type StatusMessages,
  type StatusTopic,
} from "../generated/protocol";

export type CommandAck = CommandAckStatus;
export type ConnectionState = "connecting" | "open" | "closed";

export interface RawMessage {
  channel: Channel;
  data: Uint8Array;
  /** Relay log time in nanoseconds. */
  timestamp: bigint;
}
export type RawListener = (message: RawMessage) => void;

const ACK_TOPIC = "/status/command_ack";
const ACK_TIMEOUT_MS = 1000;
const BACKOFF_MIN_MS = 500;
const BACKOFF_MAX_MS = 5000;
// The Foxglove SDK server answers 400 unless "foxglove.sdk.v1" is offered. The client library
// speaks the same wire protocol, so offer both.
const SUBPROTOCOLS = ["foxglove.sdk.v1", FoxgloveClient.SUPPORTED_SUBPROTOCOL];

export function relayUrl(): string {
  const fromEnv = import.meta.env.VITE_WS_URL as string | undefined;
  if (fromEnv) return fromEnv;
  const host = location.hostname.includes(":") ? `[${location.hostname}]` : location.hostname;
  return `ws://${host || "localhost"}:8765`;
}

interface PendingAck {
  topic: string;
  resolve: (ack: CommandAck) => void;
  reject: (err: Error) => void;
  timer: ReturnType<typeof setTimeout>;
}

class Connection {
  #url = relayUrl();
  #client: FoxgloveClient | null = null;
  #state: ConnectionState = "closed";
  #attempt = 0;
  #reconnectTimer: ReturnType<typeof setTimeout> | null = null;
  #lastMessageAt = 0;
  #canPublish = false;

  #channels = new Map<string, Channel>();
  #listeners = new Map<string, Set<RawListener>>();
  #subIdByTopic = new Map<string, number>();
  #topicBySubId = new Map<number, string>();
  #clientChannels = new Map<string, number>();
  #pending: PendingAck[] = [];

  #stateListeners = new Set<(state: ConnectionState) => void>();
  #channelListeners = new Set<(channels: ReadonlyMap<string, Channel>) => void>();
  #decoder = new TextDecoder();

  constructor() {
    this.#listen(ACK_TOPIC, (m) => this.#onAck(this.#parse<CommandAck>(m)));
  }

  get state(): ConnectionState {
    return this.#state;
  }

  get channels(): ReadonlyMap<string, Channel> {
    return this.#channels;
  }

  start(): void {
    if (this.#client || this.#reconnectTimer) return;
    this.#connect();
    document.addEventListener("visibilitychange", () => {
      if (document.visibilityState !== "visible") return;
      // Safari can hand back a dead socket after the screen unlocks. With status at 10 Hz,
      // three quiet seconds means the socket is gone even if it still says open.
      if (this.#state === "open" && performance.now() - this.#lastMessageAt > 3000) {
        this.#client?.close();
      } else if (this.#state === "closed") {
        this.#reconnectNow();
      }
    });
  }

  onState(cb: (state: ConnectionState) => void): () => void {
    this.#stateListeners.add(cb);
    cb(this.#state);
    return () => this.#stateListeners.delete(cb);
  }

  onChannels(cb: (channels: ReadonlyMap<string, Channel>) => void): () => void {
    this.#channelListeners.add(cb);
    cb(this.#channels);
    return () => this.#channelListeners.delete(cb);
  }

  /** Typed subscription to a status topic. Payloads are JSON. */
  subscribe<T extends StatusTopic>(topic: T, cb: (m: StatusMessages[T]) => void): () => void {
    return this.#listen(topic, (m) => {
      const parsed = this.#parse<StatusMessages[T]>(m);
      if (parsed !== null) cb(parsed);
    });
  }

  /** Subscription to any topic, delivering the raw payload bytes. */
  subscribeRaw(topic: string, cb: RawListener): () => void {
    return this.#listen(topic, cb);
  }

  /** Subscription to any JSON topic. Messages that fail to parse are dropped. */
  subscribeJson<T>(topic: string, cb: (m: T, raw: RawMessage) => void): () => void {
    return this.#listen(topic, (m) => {
      const parsed = this.#parse<T>(m);
      if (parsed !== null) cb(parsed, m);
    });
  }

  /** Publishes a command. Resolves on its ack, or rejects with "not delivered" after 1 s. */
  send<T extends CommandTopic>(topic: T, payload: CommandPayloads[T]): Promise<CommandAck> {
    const channelId = this.#clientChannels.get(topic);
    if (!this.#client || this.#state !== "open") {
      return Promise.reject(new Error("not connected"));
    }
    if (channelId === undefined) {
      return Promise.reject(new Error("relay does not accept commands"));
    }
    return new Promise<CommandAck>((resolve, reject) => {
      const pending: PendingAck = {
        topic,
        resolve,
        reject,
        timer: setTimeout(() => {
          this.#pending = this.#pending.filter((p) => p !== pending);
          reject(new Error("not delivered"));
        }, ACK_TIMEOUT_MS),
      };
      this.#pending.push(pending);
      try {
        this.#client!.sendMessage(channelId, new TextEncoder().encode(JSON.stringify(payload)));
      } catch (err) {
        clearTimeout(pending.timer);
        this.#pending = this.#pending.filter((p) => p !== pending);
        reject(err instanceof Error ? err : new Error(String(err)));
      }
    });
  }

  #listen(topic: string, cb: RawListener): () => void {
    let set = this.#listeners.get(topic);
    if (!set) {
      set = new Set();
      this.#listeners.set(topic, set);
    }
    set.add(cb);
    this.#syncSubscription(topic);
    return () => {
      set.delete(cb);
      if (set.size === 0) {
        this.#listeners.delete(topic);
        this.#syncSubscription(topic);
      }
    };
  }

  #parse<T>(m: RawMessage): T | null {
    try {
      return JSON.parse(this.#decoder.decode(m.data)) as T;
    } catch {
      return null;
    }
  }

  /** Subscribes or unsubscribes one topic so it matches the listener set and the channel list. */
  #syncSubscription(topic: string): void {
    const client = this.#client;
    if (!client || this.#state !== "open") return;
    const wanted = (this.#listeners.get(topic)?.size ?? 0) > 0;
    const channel = this.#channels.get(topic);
    const subId = this.#subIdByTopic.get(topic);
    if (wanted && channel && subId === undefined) {
      const id = client.subscribe(channel.id);
      this.#subIdByTopic.set(topic, id);
      this.#topicBySubId.set(id, topic);
    } else if ((!wanted || !channel) && subId !== undefined) {
      if (channel) client.unsubscribe(subId);
      this.#subIdByTopic.delete(topic);
      this.#topicBySubId.delete(subId);
    }
  }

  #connect(): void {
    this.#reconnectTimer = null;
    this.#setState("connecting");
    let ws: WebSocket;
    try {
      ws = new WebSocket(this.#url, SUBPROTOCOLS);
    } catch {
      this.#setState("closed");
      this.#scheduleReconnect();
      return;
    }
    // The DOM send() signature is narrower than IWebSocket's; the client only sends ArrayBuffers.
    const client = new FoxgloveClient({ ws: ws as unknown as IWebSocket });
    this.#client = client;

    client.on("open", () => {
      if (this.#client !== client) return;
      this.#attempt = 0;
      this.#lastMessageAt = performance.now();
      this.#setState("open");
    });
    client.on("serverInfo", (info) => {
      if (this.#client !== client) return;
      this.#canPublish = info.capabilities.includes("clientPublish");
      this.#advertiseCommands(client);
    });
    client.on("advertise", (channels) => {
      if (this.#client !== client) return;
      for (const ch of channels) {
        const old = this.#channels.get(ch.topic);
        if (old && old.id !== ch.id) this.#dropSubscription(ch.topic);
        this.#channels.set(ch.topic, ch);
      }
      for (const ch of channels) this.#syncSubscription(ch.topic);
      this.#emitChannels();
    });
    client.on("unadvertise", (ids) => {
      if (this.#client !== client) return;
      const removed = new Set(ids);
      for (const [topic, ch] of this.#channels) {
        if (!removed.has(ch.id)) continue;
        this.#dropSubscription(topic);
        this.#channels.delete(topic);
      }
      this.#emitChannels();
    });
    client.on("message", (event: MessageData) => {
      if (this.#client !== client) return;
      this.#lastMessageAt = performance.now();
      const topic = this.#topicBySubId.get(event.subscriptionId);
      if (!topic) return;
      const channel = this.#channels.get(topic);
      const listeners = this.#listeners.get(topic);
      if (!channel || !listeners) return;
      const view = event.data;
      const message: RawMessage = {
        channel,
        data: new Uint8Array(view.buffer, view.byteOffset, view.byteLength),
        timestamp: event.timestamp,
      };
      for (const cb of listeners) {
        try {
          cb(message);
        } catch (err) {
          console.error(`listener for ${topic} threw`, err);
        }
      }
    });
    client.on("close", () => {
      if (this.#client !== client) return;
      this.#teardown();
      this.#scheduleReconnect();
    });
    client.on("error", () => {
      // "close" always follows, and it does the cleanup.
    });
  }

  #advertiseCommands(client: FoxgloveClient): void {
    this.#clientChannels.clear();
    if (!this.#canPublish) {
      console.warn("relay does not advertise clientPublish; commands are disabled");
      return;
    }
    for (const [topic, schemaName] of Object.entries(COMMAND_SCHEMAS)) {
      const id = client.advertise({ topic, encoding: "json", schemaName });
      this.#clientChannels.set(topic, id);
    }
  }

  #dropSubscription(topic: string): void {
    const subId = this.#subIdByTopic.get(topic);
    if (subId === undefined) return;
    this.#subIdByTopic.delete(topic);
    this.#topicBySubId.delete(subId);
  }

  #onAck(ack: CommandAck | null): void {
    if (!ack) return;
    const index = this.#pending.findIndex((p) => p.topic === ack.topic);
    if (index < 0) return;
    const [pending] = this.#pending.splice(index, 1);
    clearTimeout(pending.timer);
    pending.resolve(ack);
  }

  #teardown(): void {
    this.#client = null;
    this.#canPublish = false;
    this.#channels.clear();
    this.#subIdByTopic.clear();
    this.#topicBySubId.clear();
    this.#clientChannels.clear();
    for (const p of this.#pending) {
      clearTimeout(p.timer);
      p.reject(new Error("not delivered"));
    }
    this.#pending = [];
    this.#setState("closed");
    this.#emitChannels();
  }

  #scheduleReconnect(): void {
    if (this.#reconnectTimer) return;
    const delay = Math.min(BACKOFF_MAX_MS, BACKOFF_MIN_MS * 2 ** this.#attempt);
    this.#attempt += 1;
    this.#reconnectTimer = setTimeout(() => this.#connect(), delay);
  }

  #reconnectNow(): void {
    if (this.#reconnectTimer) clearTimeout(this.#reconnectTimer);
    this.#reconnectTimer = null;
    this.#attempt = 0;
    this.#connect();
  }

  #setState(state: ConnectionState): void {
    if (this.#state === state) return;
    this.#state = state;
    for (const cb of this.#stateListeners) cb(state);
  }

  #emitChannels(): void {
    for (const cb of this.#channelListeners) cb(this.#channels);
  }
}

export const connection = new Connection();
