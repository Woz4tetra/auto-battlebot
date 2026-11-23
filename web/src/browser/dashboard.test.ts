import { FoxgloveServer } from "@foxglove/ws-protocol";
import { once } from "node:events";
import { existsSync, mkdtempSync, readFileSync, rmSync } from "node:fs";
import { createServer, type Server } from "node:http";
import type { AddressInfo } from "node:net";
import { tmpdir } from "node:os";
import { extname, join, normalize, resolve } from "node:path";
import { fileURLToPath } from "node:url";
import protobuf from "protobufjs";
import "protobufjs/ext/descriptor";
import puppeteer, { type Browser, type Page } from "puppeteer-core";
import { build } from "vite";
import { afterAll, beforeAll, describe, expect, it } from "vitest";
import { WebSocketServer } from "ws";
import type { SystemStatus, TracksStatus } from "../generated/protocol";

const CHROME = process.env.CHROME_PATH ?? "/usr/bin/google-chrome";
const WEB_DIR = resolve(fileURLToPath(new URL("../..", import.meta.url)));
const PUBLISH_MS = 100;
const TYPES: Record<string, string> = {
  ".html": "text/html",
  ".js": "text/javascript",
  ".css": "text/css",
  ".svg": "image/svg+xml",
  ".woff2": "font/woff2",
};

const system: SystemStatus = {
  auto_tracking: true,
  autonomy_enabled: false,
  behavior_mode: "attack",
  camera_ok: true,
  camera_recording: false,
  compute_mode: "MAXN",
  initialized: true,
  loop_rate_hz: 250,
  mcap_recording: false,
  selected_opponent_count: 1,
  transmitter_connected: true,
  transmitter_receiving: true,
  uptime_s: 12,
};

const tracks: TracksStatus = {
  camera_marker: [],
  field_outline: [],
  field_x: 2.4,
  field_y: 2.4,
  opponents_seen: 1,
  our_robot_seen: true,
  robots: [
    { id: "our_robot", label: "MR_STABS_MK2", ours: true, x: -0.5, y: 0.2, yaw: 0.3 },
    { id: "opponent_1", label: "OPPONENT", ours: false, x: 0.6, y: -0.4, yaw: 1.0 },
  ],
};

function compressedImageSchema(): { type: protobuf.Type; schema: string } {
  const root = protobuf.Root.fromJSON({
    nested: {
      google: {
        nested: {
          protobuf: {
            nested: {
              Timestamp: {
                fields: { seconds: { type: "int64", id: 1 }, nanos: { type: "int32", id: 2 } },
              },
            },
          },
        },
      },
      foxglove: {
        nested: {
          CompressedImage: {
            fields: {
              timestamp: { type: "google.protobuf.Timestamp", id: 1 },
              data: { type: "bytes", id: 2 },
              format: { type: "string", id: 3 },
              frame_id: { type: "string", id: 4 },
            },
          },
        },
      },
    },
  });
  const descriptor = (
    root as unknown as { toDescriptor(syntax: string): protobuf.Message }
  ).toDescriptor("proto3");
  const set = (descriptor.constructor as unknown as typeof protobuf.Message).encode(descriptor);
  return {
    type: root.lookupType("foxglove.CompressedImage"),
    schema: Buffer.from(set.finish()).toString("base64"),
  };
}

function serveStatic(dist: string): Promise<Server> {
  const server = createServer((request, response) => {
    const path = (request.url ?? "/").split("?")[0];
    if (path === "/healthz") {
      response.writeHead(200, { "Content-Type": "application/json" });
      response.end(JSON.stringify({ app_connected: true }));
      return;
    }
    const file = normalize(join(dist, path === "/" ? "index.html" : path));
    const target = file.startsWith(dist) && existsSync(file) ? file : join(dist, "index.html");
    response.writeHead(200, {
      "Content-Type": TYPES[extname(target)] ?? "application/octet-stream",
    });
    response.end(readFileSync(target));
  });
  return new Promise((done) => server.listen(0, "127.0.0.1", () => done(server)));
}

class FakeRelay {
  readonly wss: WebSocketServer;
  readonly server = new FoxgloveServer({ name: "fake relay" });
  readonly #timer: ReturnType<typeof setInterval>;
  #jpeg: Uint8Array | null = null;

  constructor() {
    this.wss = new WebSocketServer({
      host: "127.0.0.1",
      port: 0,
      handleProtocols: (protocols) =>
        protocols.has(FoxgloveServer.SUPPORTED_SUBPROTOCOL)
          ? FoxgloveServer.SUPPORTED_SUBPROTOCOL
          : false,
    });
    this.wss.on("connection", (ws, request) =>
      this.server.handleConnection(
        ws as unknown as Parameters<FoxgloveServer["handleConnection"]>[0],
        request.url ?? "",
      ),
    );
    const json = (topic: string, schemaName: string) =>
      this.server.addChannel({ topic, encoding: "json", schemaName, schema: "{}" });
    const channels = {
      system: json("/status/system", "auto_battlebot.status.System"),
      tracks: json("/status/tracks", "auto_battlebot.status.Tracks"),
      diagnostics: json("/diagnostics/runner", "auto_battlebot.Diagnostics"),
    };
    const image = compressedImageSchema();
    const preview = this.server.addChannel({
      topic: "/camera/preview",
      encoding: "protobuf",
      schemaName: "foxglove.CompressedImage",
      schema: image.schema,
      schemaEncoding: "protobuf",
    });
    const encoder = new TextEncoder();
    let loop = 0;
    this.#timer = setInterval(() => {
      const now = BigInt(Date.now()) * 1_000_000n;
      loop += 1;
      const send = (id: number, payload: unknown) =>
        this.server.sendMessage(id, now, encoder.encode(JSON.stringify(payload)));
      send(channels.system, system);
      send(channels.tracks, tracks);
      send(channels.diagnostics, {
        pipeline: { level: 1, message: "slow frame", age_ms: 1, held: false, values: { loop } },
      });
      if (this.#jpeg) {
        const message = image.type.create({
          timestamp: { seconds: Math.floor(Date.now() / 1000), nanos: 0 },
          data: this.#jpeg,
          format: "jpeg",
          frame_id: "camera",
        });
        this.server.sendMessage(preview, now, new Uint8Array(image.type.encode(message).finish()));
      }
    }, PUBLISH_MS);
  }

  url(): string {
    return `ws://127.0.0.1:${(this.wss.address() as AddressInfo).port}`;
  }

  publishJpeg(bytes: Uint8Array): void {
    this.#jpeg = bytes;
  }

  close(): void {
    clearInterval(this.#timer);
    for (const client of this.wss.clients) client.terminate();
    this.wss.close();
  }
}

describe.skipIf(!existsSync(CHROME))("dashboard against a fake relay", () => {
  let dist = "";
  let relay: FakeRelay;
  let web: Server;
  let browser: Browser;
  let page: Page;
  let origin = "";

  beforeAll(async () => {
    relay = new FakeRelay();
    await once(relay.wss, "listening");
    dist = mkdtempSync(join(tmpdir(), "dashboard_page_"));
    process.env.VITE_WS_URL = relay.url();
    try {
      await build({
        root: WEB_DIR,
        configFile: join(WEB_DIR, "vite.config.ts"),
        logLevel: "silent",
        build: { outDir: dist, emptyOutDir: true },
      });
    } finally {
      delete process.env.VITE_WS_URL;
    }
    web = await serveStatic(dist);
    origin = `http://127.0.0.1:${(web.address() as AddressInfo).port}`;
    browser = await puppeteer.launch({ executablePath: CHROME, args: ["--no-sandbox"] });
    page = await browser.newPage();
    await page.setViewport({ width: 1280, height: 800 });
  }, 120_000);

  afterAll(async () => {
    await browser?.close();
    relay?.close();
    web?.closeAllConnections();
    web?.close();
    if (dist) rmSync(dist, { recursive: true, force: true });
  });

  it("connects with no alert banner while the app sends status", async () => {
    await page.goto(`${origin}/`, { waitUntil: "domcontentloaded" });
    await page.waitForFunction(() => document.querySelector(".topdown") !== null);
    await page.waitForFunction(() => !document.querySelector('[role="alert"]'), {
      timeout: 15_000,
    });
  }, 30_000);

  it("draws each tracked robot on the top-down view", async () => {
    await page.waitForSelector('svg[aria-label="Top-down field view with robot tracks"] .ours');
    const shapes = await page.$$eval(
      'svg[aria-label="Top-down field view with robot tracks"] g',
      (groups) => groups.map((g) => g.getAttribute("class") ?? ""),
    );
    expect(shapes.filter((c) => c.includes("ours"))).toHaveLength(1);
    expect(shapes.length).toBeGreaterThanOrEqual(2);
  });

  it("paints the JPEG preview and drops the no-preview tag", async () => {
    expect(await page.$(".nosignal")).not.toBeNull();
    const jpeg = await page.evaluate(() => {
      const canvas = document.createElement("canvas");
      canvas.width = 64;
      canvas.height = 36;
      const context = canvas.getContext("2d")!;
      context.fillStyle = "#c03030";
      context.fillRect(0, 0, 64, 36);
      return canvas.toDataURL("image/jpeg").split(",")[1];
    });
    relay.publishJpeg(Uint8Array.from(Buffer.from(jpeg, "base64")));
    await page.waitForFunction(() => !document.querySelector(".nosignal"), { timeout: 15_000 });
  }, 30_000);

  it("lists diagnostics modules and plots a selected value", async () => {
    await page.evaluate(() => {
      const tab = [...document.querySelectorAll('nav[aria-label="Sections"] button')].find((b) =>
        /diag/i.test(b.textContent ?? ""),
      ) as HTMLButtonElement | undefined;
      tab?.click();
    });
    await page.waitForFunction(
      () =>
        [...document.querySelectorAll("button.module")].some((b) =>
          b.textContent?.includes("runner"),
        ),
      { timeout: 15_000 },
    );
    const module = await page.$$eval("button.module", (buttons) =>
      buttons.map((b) => b.textContent?.replace(/\s+/g, " ").trim() ?? ""),
    );
    expect(module.join(" ")).toContain("slow frame");
    await page.evaluate(() => {
      const button = [...document.querySelectorAll("button.module")].find((b) =>
        b.textContent?.includes("runner"),
      ) as HTMLButtonElement;
      button.click();
    });
    await page.waitForFunction(
      () => {
        const line = document.querySelector('svg[aria-label="Recent values"] polyline');
        return (line?.getAttribute("points") ?? "").trim().split(/\s+/).length >= 2;
      },
      { timeout: 15_000 },
    );
  }, 30_000);

  it("shows the offline banner once the relay goes away", async () => {
    relay.close();
    await page.waitForFunction(
      () => document.querySelector('[role="alert"]')?.textContent?.includes("OFFLINE") ?? false,
      { timeout: 20_000 },
    );
  }, 30_000);
});
