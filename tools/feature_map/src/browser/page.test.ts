import { existsSync, mkdtempSync, readFileSync, rmSync } from "node:fs";
import { createServer, type Server } from "node:http";
import { tmpdir } from "node:os";
import { extname, join, normalize, resolve } from "node:path";
import { fileURLToPath } from "node:url";
import puppeteer, { type Browser, type Page } from "puppeteer-core";
import { build } from "vite";
import { afterAll, beforeAll, describe, expect, it } from "vitest";
import type { Feature, FeatureMap, ProofEntry } from "../lib/types";

const CHROME = process.env.CHROME_PATH ?? "/usr/bin/google-chrome";
const TOOL_DIR = resolve(fileURLToPath(new URL("../..", import.meta.url)));
const TYPES: Record<string, string> = {
  ".html": "text/html",
  ".js": "text/javascript",
  ".css": "text/css",
};

function proof(id: string, state: ProofEntry["state"], kind: ProofEntry["kind"]): ProofEntry {
  return {
    id,
    state,
    last_status: state === "passing" ? "passed" : null,
    last_seconds: 0.25,
    last_stamp: null,
    last_pass_commit: state === "passing" ? "abc123" : null,
    last_pass_stamp: null,
    flakes: 0,
    changed: [],
    kind,
    runner: id.split(":")[0],
    command: `scripts/verify --only ${id}`,
    modules: ["src/transmitter/"],
  };
}

const transmit: Feature = {
  key: "command_transmit",
  name: "Command transmit to the robot",
  status: "draft",
  stage: "transmit",
  stages: ["transmit"],
  reach: ["topic:/robot/espnow_telemetry"],
  variants: [
    {
      type: "EspNowTransmitter",
      proofs: [proof("gtest:EspNow.Sends", "stale", "variant")],
      state: "stale",
      config: {
        section: "transmitter",
        fields: [{ section: "transmitter", key: "rate_hz", default: "100.0", required: false }],
        profiles: ["mr_stabs_mk2_aer_zed_box"],
      },
    },
  ],
  code: ["src/transmitter/"],
  touches: [],
  proofs: [proof("gtest:EspNow.SubstitutesUnderSa", "passing", "entry")],
  supporting: [],
  links: [{ name: "espnow", grade: "passed", stamp: null }],
  docs: [],
  state: "stale",
  commits: [{ sha: "abc123", date: "2026-11-22", subject: "Send at 100 Hz" }],
  churn: 2,
  in_diff: false,
};

const fixture: FeatureMap = {
  generated: "",
  head: "abc123",
  dirty: "",
  enforce: false,
  stages: [
    {
      id: "transmit",
      label: "Transmit",
      proven: 0,
      stale: 1,
      failed: 0,
      gap: 0,
      features: ["command_transmit"],
    },
    { id: "platform", label: "Platform", proven: 0, stale: 0, failed: 0, gap: 0, features: [] },
  ],
  features: [transmit],
  reach: [],
  impact: { modules: { "src/transmitter/": ["command_transmit"] }, infrastructure: [] },
  diff: { files: [], features: [], would_run: [], command: "scripts/verify --changed" },
  check: { errors: [], gaps: [] },
  runs: [{ stamp: "2026-11-22T14-00-00.000000Z", commit: "abc123", tier: "t", results: 2 }],
};

function serve(dist: string): Promise<Server> {
  const server = createServer((request, response) => {
    const path = (request.url ?? "/").split("?")[0];
    if (path === "/api/map") {
      response.writeHead(200, { "Content-Type": "application/json" });
      response.end(JSON.stringify(fixture));
      return;
    }
    if (path === "/api/events") {
      response.writeHead(200, { "Content-Type": "text/event-stream" });
      response.write(": open\n\n");
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

describe.skipIf(!existsSync(CHROME))("feature map page", () => {
  let dist = "";
  let server: Server;
  let browser: Browser;
  let page: Page;
  let origin = "";

  beforeAll(async () => {
    dist = mkdtempSync(join(tmpdir(), "feature_map_page_"));
    await build({
      root: TOOL_DIR,
      configFile: join(TOOL_DIR, "vite.config.ts"),
      logLevel: "silent",
      build: { outDir: dist, emptyOutDir: true },
    });
    server = await serve(dist);
    const address = server.address();
    origin = `http://127.0.0.1:${typeof address === "object" && address ? address.port : 0}`;
    browser = await puppeteer.launch({ executablePath: CHROME, args: ["--no-sandbox"] });
    await browser
      .defaultBrowserContext()
      .overridePermissions(origin, [
        "clipboard-read",
        "clipboard-write",
        "clipboard-sanitized-write",
      ]);
    page = await browser.newPage();
    await page.setViewport({ width: 1400, height: 900 });
  }, 60_000);

  afterAll(async () => {
    await browser?.close();
    server?.closeAllConnections();
    server?.close();
    if (dist) rmSync(dist, { recursive: true, force: true });
  });

  it("shows stage counts, the graph, the side panel, and copies a command", async () => {
    const errors: string[] = [];
    page.on("pageerror", (error) => errors.push(String(error)));
    await page.goto(`${origin}/?debug`, { waitUntil: "domcontentloaded" });
    await page.waitForSelector(".stage");
    const stages = await page.$$eval(".stage .label", (nodes) => nodes.map((n) => n.textContent));
    expect(stages).toEqual(["Transmit", "Platform"]);
    const lane = await page.$eval(".lane", (n) => n.getBoundingClientRect().bottom);
    const chip = await page.$eval(".lanes .chip", (n) => n.getBoundingClientRect().bottom);
    expect(lane).toBeGreaterThan(chip);
    expect(await page.$eval("nav .meta", (n) => n.textContent)).toContain("last run");

    await page.click(".stage");
    await page.waitForFunction(() => {
      const cy = (window as unknown as { featureMapCy?: { nodes(): { length: number } } })
        .featureMapCy;
      return cy !== undefined && cy.nodes().length > 0;
    });
    const position = await page.evaluate(() => {
      type Node = { renderedPosition(): { x: number; y: number } };
      type Cy = { getElementById(id: string): Node; nodes(): unknown[]; container(): Element };
      const cy = (window as unknown as { featureMapCy: Cy }).featureMapCy;
      const box = cy.container().getBoundingClientRect();
      const at = cy.getElementById("feature:command_transmit").renderedPosition();
      return { x: box.left + at.x, y: box.top + at.y, count: cy.nodes().length };
    });
    expect(position.count).toBe(3);
    await page.mouse.click(position.x, position.y);
    await page.waitForSelector("aside h3");
    expect(await page.$eval("aside h3", (n) => n.textContent)).toContain(
      "Command transmit to the robot",
    );
    const panel = await page.$eval("aside", (n) => n.textContent ?? "");
    expect(panel).toContain("selected by mr_stabs_mk2_aer_zed_box");
    expect(panel).toContain("espnow: passed");
    expect(panel).toContain("2 since last green");

    const rerun = await page.$$eval("aside .actions button", (buttons) =>
      buttons.map((b) => b.getAttribute("title")),
    );
    expect(rerun).toContain("scripts/verify --only gtest:EspNow.Sends");
    await page.bringToFront();
    await page.$eval("aside .actions button:last-child", (b) => (b as HTMLButtonElement).click());
    await page.waitForFunction(
      () => document.querySelector("aside .actions button:last-child")?.textContent === "copied",
    );
    expect(await page.evaluate(() => navigator.clipboard.readText())).toBe(
      "scripts/verify --only gtest:EspNow.Sends",
    );
    expect(errors).toEqual([]);
  }, 60_000);
});
