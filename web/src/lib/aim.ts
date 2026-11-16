import type { SetManualTargetCommand } from "../generated/protocol";
import { status } from "./status.svelte";

const DRAG_SEND_INTERVAL_MS = 100;

export interface AimParams {
  enabled: boolean;
  point: (e: PointerEvent) => SetManualTargetCommand | null;
}

export function aimDrag(node: HTMLElement | SVGElement, params: AimParams) {
  let current = params;
  let dragging = false;
  let lastSent = 0;
  let pending: SetManualTargetCommand | null = null;
  let timer: ReturnType<typeof setTimeout> | null = null;

  const send = (target: SetManualTargetCommand) => {
    lastSent = Date.now();
    void status.command("/command/set_manual_target", target);
  };
  const flush = () => {
    timer = null;
    if (pending) send(pending);
    pending = null;
  };
  const throttled = (target: SetManualTargetCommand) => {
    const wait = DRAG_SEND_INTERVAL_MS - (Date.now() - lastSent);
    if (wait <= 0 && !timer) {
      send(target);
      return;
    }
    pending = target;
    timer ??= setTimeout(flush, Math.max(0, wait));
  };

  const down = (e: PointerEvent) => {
    if (!current.enabled) return;
    const target = current.point(e);
    if (!target) return;
    dragging = true;
    node.setPointerCapture(e.pointerId);
    send(target);
  };
  const move = (e: PointerEvent) => {
    if (!dragging) return;
    const target = current.point(e);
    if (target) throttled(target);
  };
  const up = (e: PointerEvent) => {
    if (!dragging) return;
    dragging = false;
    if (timer) clearTimeout(timer);
    timer = null;
    pending = null;
    const target = current.point(e);
    if (target) send(target);
  };
  const cancel = () => {
    dragging = false;
  };

  const el = node as HTMLElement;
  el.addEventListener("pointerdown", down);
  el.addEventListener("pointermove", move);
  el.addEventListener("pointerup", up);
  el.addEventListener("pointercancel", cancel);
  return {
    update(next: AimParams) {
      current = next;
      if (!next.enabled) dragging = false;
    },
    destroy() {
      if (timer) clearTimeout(timer);
      el.removeEventListener("pointerdown", down);
      el.removeEventListener("pointermove", move);
      el.removeEventListener("pointerup", up);
      el.removeEventListener("pointercancel", cancel);
    },
  };
}
