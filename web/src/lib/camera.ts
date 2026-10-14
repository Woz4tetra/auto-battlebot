// Camera profile helpers shared by the sidebar tile and the CAMERA tab. Values travel in ZED SDK
// units (exposure in microseconds, analog gain in milli-dB); these format them for display.

import type { CameraSettings, CameraSettingLimits } from "../generated/protocol";
import { status } from "./status.svelte";

export const DEFAULT_PROFILE = "DEFAULT";

/** One slider on the CAMERA tab. `auto` names the switch that locks it while on. */
export interface Control {
  key: Exclude<keyof CameraSettingLimits, "anti_banding">;
  label: string;
  step: number;
  auto?: "auto_exposure" | "auto_white_balance";
  format: (v: number) => string;
}

export const CONTROLS: Control[] = [
  {
    key: "exposure_us",
    label: "EXPOSURE TIME",
    step: 100,
    auto: "auto_exposure",
    format: (v) => `${(v / 1000).toFixed(2)} ms`,
  },
  {
    key: "analog_gain_mdb",
    label: "ANALOG GAIN",
    step: 500,
    auto: "auto_exposure",
    format: (v) => `${(v / 1000).toFixed(1)} dB`,
  },
  {
    key: "digital_gain",
    label: "DIGITAL GAIN",
    step: 1,
    auto: "auto_exposure",
    format: (v) => `${v}×`,
  },
  {
    key: "white_balance_k",
    label: "WHITE BALANCE",
    step: 100,
    auto: "auto_white_balance",
    format: (v) => `${v} K`,
  },
  { key: "saturation", label: "SATURATION", step: 1, format: (v) => String(v) },
  { key: "gamma", label: "GAMMA", step: 1, format: (v) => String(v) },
  { key: "sharpness", label: "SHARPNESS", step: 1, format: (v) => String(v) },
];

export const ANTI_BANDING = [
  { value: "off", label: "OFF" },
  { value: "auto", label: "AUTO" },
  { value: "hz_50", label: "50" },
  { value: "hz_60", label: "60" },
] as const;

/** One-line summary of the exposure, gain, and white balance a profile runs. */
export function summary(s: CameraSettings): string {
  const exposure = s.auto_exposure
    ? "AUTO EXP"
    : `${(s.exposure_us / 1000).toFixed(1)} MS · ${(s.analog_gain_mdb / 1000).toFixed(0)} DB`;
  const wb = s.auto_white_balance ? "AUTO WB" : `${s.white_balance_k} K`;
  return `${exposure} · ${wb}`;
}

/** Motion blur of a robot crossing the frame at `speed_mps`, in mm. */
export function blurMm(exposure_us: number, speed_mps = 3): number {
  return Math.round((exposure_us / 1e6) * speed_mps * 1000);
}

export function applyProfile(name: string) {
  return status.command("/command/camera_profile", { action: "apply", name });
}
