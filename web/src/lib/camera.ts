// Camera profile helpers shared by the sidebar tile and the CAMERA tab. Values travel in ZED SDK
// units (exposure in microseconds, analog gain in milli-dB); these format them for display.

import type { CameraSettings, CameraSettingLimits } from "../generated/protocol";
import { status } from "./status.svelte";

export const DEFAULT_PROFILE = "DEFAULT";

export interface Control {
  key: Exclude<keyof CameraSettingLimits, "anti_banding">;
  label: string;
  step: number;
  autoSwitchKey?: "auto_exposure" | "auto_white_balance";
  format: (v: number) => string;
}

export const CONTROLS: Control[] = [
  {
    key: "exposure_us",
    label: "EXPOSURE TIME",
    step: 100,
    autoSwitchKey: "auto_exposure",
    format: (v) => `${(v / 1000).toFixed(2)} ms`,
  },
  {
    key: "analog_gain_mdb",
    label: "ANALOG GAIN",
    step: 500,
    autoSwitchKey: "auto_exposure",
    format: (v) => `${(v / 1000).toFixed(1)} dB`,
  },
  {
    key: "digital_gain",
    label: "DIGITAL GAIN",
    step: 1,
    autoSwitchKey: "auto_exposure",
    format: (v) => `${v}×`,
  },
  {
    key: "white_balance_k",
    label: "WHITE BALANCE",
    step: 100,
    autoSwitchKey: "auto_white_balance",
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

export function summary(s: CameraSettings): string {
  const exposure = s.auto_exposure
    ? "AUTO EXP"
    : `${(s.exposure_us / 1000).toFixed(1)} MS · ${(s.analog_gain_mdb / 1000).toFixed(0)} DB`;
  const wb = s.auto_white_balance ? "AUTO WB" : `${s.white_balance_k} K`;
  return `${exposure} · ${wb}`;
}

export function motionBlurMm(exposure_us: number, speedMps = 3): number {
  return Math.round((exposure_us / 1e6) * speedMps * 1000);
}

export function applyProfile(name: string) {
  return status.command("/command/camera_profile", { action: "apply", name });
}
