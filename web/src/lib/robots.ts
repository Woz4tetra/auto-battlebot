// One color per robot, shared by the camera view and the top-down view.
//
// Detections carry a label and no track id, so the key is the label. The color comes from a hash
// of it, so the iPad and the phone agree without talking to each other; a label that hashes onto a
// color another label already took this session moves to the next free one. Our robot always gets
// its own color outside the palette. Two tracked robots that share a label (two generic
// "opponent"s) are told apart in the top-down view by track id; in the camera view they share the
// label's color, since a detection cannot say which track it is.

const OURS = "#2ec4e6";
const PALETTE = ["#ff6b4a", "#b77cff", "#7bd84a", "#ff5fb0", "#f2c94c", "#4f8bff", "#3ddbb3"];

const assigned = new Map<string, string>();

function hash(text: string): number {
  // FNV-1a, 32 bit.
  let h = 0x811c9dc5;
  for (let i = 0; i < text.length; i++) {
    h ^= text.charCodeAt(i);
    h = Math.imul(h, 0x01000193) >>> 0;
  }
  return h;
}

export function robotColor(key: string, ours = false): string {
  if (ours) return OURS;
  let color = assigned.get(key);
  if (!color) {
    const taken = new Set(assigned.values());
    const start = hash(key) % PALETTE.length;
    color = PALETTE[start];
    for (let i = 0; i < PALETTE.length; i++) {
      const candidate = PALETTE[(start + i) % PALETTE.length];
      if (!taken.has(candidate)) {
        color = candidate;
        break;
      }
    }
    assigned.set(key, color);
  }
  return color;
}

/** Top-down key: the label, or label and track id when another tracked robot shares the label. */
export function trackKey(robot: { id: string; label: string }, all: { label: string }[]): string {
  const shared = all.filter((r) => r.label === robot.label).length > 1;
  return shared ? `${robot.label}#${robot.id}` : robot.label;
}

/** On-screen name: underscores become spaces so a label can wrap and reads faster. */
export function displayLabel(label: string): string {
  return label.replace(/_/g, " ");
}
