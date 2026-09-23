// One color per robot, shared by the camera view and the top-down view.
//
// Colors come from [ui.label_colors] in config/_common.toml (the class colors training uses),
// delivered on /status/app. A label with no configured color gets one from FALLBACK, picked by a
// hash of the label so every device agrees, and skipping colors the config already uses.
//
// Detections carry a label and no track id, so the camera view colors by label. In the top-down
// view, a second or third track with the same label (two generic "opponent"s) gets a lighter
// shade of that color so the two can be told apart.
import { status } from "./status.svelte";

const FALLBACK = ["#ec4899", "#14b8a6", "#eab308", "#6366f1", "#84cc16", "#f97316", "#8b5cf6"];

function hash(text: string): number {
  // FNV-1a, 32 bit.
  let h = 0x811c9dc5;
  for (let i = 0; i < text.length; i++) {
    h ^= text.charCodeAt(i);
    h = Math.imul(h, 0x01000193) >>> 0;
  }
  return h;
}

function configured(label: string): string | undefined {
  return status.app?.label_colors.find((c) => c.label === label)?.color;
}

export function robotColor(label: string): string {
  const color = configured(label);
  if (color) return color;
  const used = new Set(status.app?.label_colors.map((c) => c.color.toLowerCase()) ?? []);
  const free = FALLBACK.filter((c) => !used.has(c));
  const pool = free.length ? free : FALLBACK;
  return pool[hash(label) % pool.length];
}

/** Mixes a "#rrggbb" color toward white by `amount` (0 to 1). */
function lighten(color: string, amount: number): string {
  const n = parseInt(color.slice(1), 16);
  const mix = (c: number) => Math.round(c + (255 - c) * amount);
  const [r, g, b] = [mix((n >> 16) & 255), mix((n >> 8) & 255), mix(n & 255)];
  return `#${((1 << 24) | (r << 16) | (g << 8) | b).toString(16).slice(1)}`;
}

/** Top-down color: the label's color, lightened for each earlier track that shares the label. */
export function trackColor(
  robot: { id: string; label: string },
  all: { id: string; label: string }[],
): string {
  const same = all.filter((r) => r.label === robot.label).map((r) => r.id);
  same.sort();
  const index = Math.max(0, same.indexOf(robot.id));
  return index === 0 ? robotColor(robot.label) : lighten(robotColor(robot.label), 0.35 * index);
}

/** On-screen name: underscores become spaces so a label can wrap and reads faster. */
export function displayLabel(label: string): string {
  return label.replace(/_/g, " ");
}
