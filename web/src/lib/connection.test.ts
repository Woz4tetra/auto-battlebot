import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";

class FakeSocket {
  static instances: FakeSocket[] = [];
  binaryType = "";
  protocol = "foxglove.websocket.v1";
  onerror: ((e: unknown) => void) | null = null;
  onopen: ((e: unknown) => void) | null = null;
  onclose: ((e: unknown) => void) | null = null;
  onmessage: ((e: unknown) => void) | null = null;
  sent: unknown[] = [];
  closed = false;

  constructor(
    public url: string,
    public protocols: string[],
  ) {
    FakeSocket.instances.push(this);
  }

  send(data: unknown): void {
    this.sent.push(typeof data === "string" ? JSON.parse(data) : data);
  }

  close(): void {
    this.closed = true;
  }

  open(): void {
    this.onopen?.({});
  }

  serverJson(message: unknown): void {
    this.onmessage?.({ data: JSON.stringify(message) });
  }

  serverData(subscriptionId: number, payload: string): void {
    const body = new TextEncoder().encode(payload);
    const buf = new ArrayBuffer(13 + body.length);
    const view = new DataView(buf);
    view.setUint8(0, 1);
    view.setUint32(1, subscriptionId, true);
    view.setBigUint64(5, 0n, true);
    new Uint8Array(buf, 13).set(body);
    this.onmessage?.({ data: buf });
  }

  serverClose(): void {
    this.onclose?.({});
  }

  subscribeIds(): number[] {
    return this.sent
      .filter((m): m is { op: string; subscriptions: { id: number }[] } => isOp(m, "subscribe"))
      .flatMap((m) => m.subscriptions.map((s) => s.id));
  }
}

function isOp(m: unknown, op: string): boolean {
  return typeof m === "object" && m !== null && (m as { op?: string }).op === op;
}

const CHANNELS = [
  { id: 7, topic: "/status/system", encoding: "json", schemaName: "s", schema: "" },
  { id: 8, topic: "/status/command_ack", encoding: "json", schemaName: "a", schema: "" },
];

let healthz: () => Promise<Response>;

async function makeConnection() {
  const mod = await import("./connection");
  return mod;
}

function lastSocket(): FakeSocket {
  return FakeSocket.instances[FakeSocket.instances.length - 1];
}

function openSocket(ws: FakeSocket): void {
  ws.open();
  ws.serverJson({ op: "serverInfo", name: "r", capabilities: [], supportedEncodings: [] });
  ws.serverJson({ op: "advertise", channels: CHANNELS });
}

describe("Connection", () => {
  beforeEach(() => {
    vi.useFakeTimers();
    FakeSocket.instances = [];
    healthz = async () => new Response(JSON.stringify({ app_connected: true }));
    vi.stubGlobal("WebSocket", FakeSocket);
    vi.stubGlobal("location", { hostname: "relay.test" });
    vi.stubGlobal("document", { addEventListener: () => {}, visibilityState: "visible" });
    vi.stubGlobal("fetch", () => healthz());
    vi.spyOn(console, "warn").mockImplementation(() => {});
  });

  afterEach(() => {
    vi.useRealTimers();
    vi.unstubAllGlobals();
    vi.restoreAllMocks();
  });

  it("offers the sdk subprotocol before the foxglove one", async () => {
    const { Connection, SUBPROTOCOLS } = await makeConnection();
    new Connection().start();
    expect(lastSocket().protocols).toEqual(SUBPROTOCOLS);
    expect(SUBPROTOCOLS).toEqual(["foxglove.sdk.v1", "foxglove.websocket.v1"]);
  });

  it("abandons a handshake that has not opened and retries", async () => {
    const { Connection, CONNECT_TIMEOUT_MS } = await makeConnection();
    const conn = new Connection();
    conn.start();
    const first = lastSocket();
    vi.advanceTimersByTime(CONNECT_TIMEOUT_MS - 1);
    expect(first.closed).toBe(false);
    expect(conn.state).toBe("connecting");
    vi.advanceTimersByTime(1);
    expect(first.closed).toBe(true);
    expect(conn.state).toBe("closed");
    vi.advanceTimersByTime(500);
    expect(FakeSocket.instances).toHaveLength(2);
    expect(conn.state).toBe("connecting");
  });

  it("keeps a handshake that opens in time", async () => {
    const { Connection, CONNECT_TIMEOUT_MS } = await makeConnection();
    const conn = new Connection();
    conn.start();
    openSocket(lastSocket());
    vi.advanceTimersByTime(CONNECT_TIMEOUT_MS * 2);
    expect(conn.state).toBe("open");
    expect(FakeSocket.instances).toHaveLength(1);
  });

  it("drops a silent socket when the probe says the app is connected", async () => {
    const { Connection, SILENCE_MS } = await makeConnection();
    healthz = async () => new Response(JSON.stringify({ app_connected: true }));
    const conn = new Connection();
    conn.start();
    const first = lastSocket();
    openSocket(first);
    await vi.advanceTimersByTimeAsync(SILENCE_MS + 1000);
    expect(first.closed).toBe(true);
    expect(FakeSocket.instances).toHaveLength(2);
  });

  it("drops a silent socket when the probe fails", async () => {
    const { Connection, SILENCE_MS } = await makeConnection();
    healthz = async () => {
      throw new Error("offline");
    };
    const conn = new Connection();
    conn.start();
    const first = lastSocket();
    openSocket(first);
    await vi.advanceTimersByTimeAsync(SILENCE_MS + 1000);
    expect(first.closed).toBe(true);
    expect(FakeSocket.instances).toHaveLength(2);
  });

  it("keeps a silent socket when the probe says the app is not connected", async () => {
    const { Connection, SILENCE_MS } = await makeConnection();
    healthz = async () => new Response(JSON.stringify({ app_connected: false }));
    const conn = new Connection();
    conn.start();
    const first = lastSocket();
    openSocket(first);
    await vi.advanceTimersByTimeAsync(SILENCE_MS + 2000);
    expect(first.closed).toBe(false);
    expect(conn.state).toBe("open");
    expect(FakeSocket.instances).toHaveLength(1);
  });

  it("keeps a socket that keeps receiving messages", async () => {
    const { Connection, SILENCE_MS } = await makeConnection();
    healthz = async () => {
      throw new Error("offline");
    };
    const conn = new Connection();
    conn.start();
    const ws = lastSocket();
    openSocket(ws);
    conn.subscribeRaw("/status/system", () => {});
    for (let i = 0; i < 10; i++) {
      await vi.advanceTimersByTimeAsync(SILENCE_MS / 2);
      ws.serverData(ws.subscribeIds()[0], "{}");
    }
    expect(ws.closed).toBe(false);
    expect(conn.state).toBe("open");
  });

  it("resubscribes listeners after a reconnect", async () => {
    const { Connection } = await makeConnection();
    const conn = new Connection();
    const seen: string[] = [];
    conn.subscribeRaw("/status/system", (m) => seen.push(new TextDecoder().decode(m.data)));
    conn.start();
    const first = lastSocket();
    openSocket(first);
    expect(first.subscribeIds()).toHaveLength(2);
    first.serverData(first.subscribeIds()[0], "one");
    first.serverClose();
    expect(conn.state).toBe("closed");
    vi.advanceTimersByTime(500);
    const second = lastSocket();
    expect(second).not.toBe(first);
    openSocket(second);
    expect(second.subscribeIds()).toHaveLength(2);
    second.serverData(second.subscribeIds()[0], "two");
    expect(seen).toEqual(["one", "two"]);
  });

  it("subscribes a listener added while open and unsubscribes when it leaves", async () => {
    const { Connection } = await makeConnection();
    const conn = new Connection();
    conn.start();
    const ws = lastSocket();
    openSocket(ws);
    const off = conn.subscribeRaw("/status/system", () => {});
    expect(ws.subscribeIds()).toHaveLength(2);
    off();
    expect(ws.sent.some((m) => isOp(m, "unsubscribe"))).toBe(true);
  });
});
