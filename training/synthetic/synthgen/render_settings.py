"""Cycles device and denoiser choices, and the cheap settings for the occlusion-only pass.

Measured on 2026-09-12 (one NHRL cage scene, 10 frames at 1280x720, one A6000): BlenderProc's
default Intel denoiser runs as a compositor node on the CPU and cost 6 s per scene, and the
occlusion-free re-render was a full-quality colour render that only ever had its instance
segmap read, 28 s per scene. Together with the CPU sitting in the Cycles device list next
to the GPUs, these were half of the scene time. Everything here requires Blender.
"""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager

import blenderproc as bproc
import bpy

from synthgen.logsetup import get_logger

logger = get_logger(__name__)

# The Cycles denoiser names the cage spec's ``render.denoiser`` accepts, plus "none".
_CYCLES_DENOISERS = {
    "OPTIX": "OPTIX",
    "INTEL": "OPENIMAGEDENOISE",
    "OPENIMAGEDENOISE": "OPENIMAGEDENOISE",
}


def _cycles_preferences() -> bpy.types.AddonPreferences:
    return bpy.context.preferences.addons["cycles"].preferences


def use_gpu_only() -> None:
    """Take the CPU out of the Cycles device list whenever a GPU is rendering.

    BlenderProc's init enables every device of the chosen type, and Blender lists the CPU
    among the OptiX devices. A 1280x720 frame is small enough that the CPU's tiles finish
    after the GPU's, so it adds wall time (50 s versus 48 s per scene on three A6000s). A
    CPU-only run keeps the CPU, since it is the only device there is.
    """
    devices = list(_cycles_preferences().devices)
    if not any(device.use and device.type != "CPU" for device in devices):
        return
    for device in devices:
        if device.type == "CPU":
            device.use = False
    logger.info("Cycles devices: %s", [device.name for device in devices if device.use] or ["CPU"])


def set_denoiser(name: str) -> None:
    """Denoise inside Cycles on the render device instead of in the compositor.

    ``name`` is a cage spec value: "OPTIX", "INTEL" or "none". OptiX needs an OptiX device,
    so a CUDA or CPU run gets OpenImageDenoise, which Blender 4.2 also runs on the GPU when
    one renders. "INTEL" maps to the same OpenImageDenoise, minus BlenderProc's compositor
    node and its CPU round trip. Going through BlenderProc first clears whatever denoiser was
    active, compositor node included.
    """
    bproc.renderer.set_denoiser(None)
    if name.lower() == "none":
        return
    wanted = _CYCLES_DENOISERS.get(name.upper())
    if wanted is None:
        raise ValueError(f"unknown denoiser {name!r}; expected one of OPTIX, INTEL, none")
    if wanted == "OPTIX" and _cycles_preferences().compute_device_type != "OPTIX":
        logger.info(
            "OptiX denoiser unavailable on %s; using OpenImageDenoise",
            _cycles_preferences().compute_device_type,
        )
        wanted = "OPENIMAGEDENOISE"
    scene = bpy.context.scene
    scene.cycles.use_denoising = True
    bpy.context.view_layer.cycles.use_denoising = True
    scene.cycles.denoiser = wanted
    scene.cycles.denoising_input_passes = "RGB_ALBEDO_NORMAL"
    scene.cycles.denoising_prefilter = "ACCURATE"


_BOUNCE_FIELDS = (
    "diffuse_bounces",
    "glossy_bounces",
    "ao_bounces_render",
    "max_bounces",
    "transmission_bounces",
    "volume_bounces",
)


@contextmanager
def occlusion_pass() -> Iterator[None]:
    """Render settings for a pass whose only output that gets read is the segmap.

    One sample, no adaptive sampling, no denoiser and camera rays only: the object-index pass
    the segmap comes from is decided by the first hit, so none of the colour work changes it.
    Measured 3.9 s instead of 28 s for ten frames, with byte-identical labels. Everything is
    put back on exit so the next colour pass renders as configured.
    """
    scene = bpy.context.scene
    cycles = scene.cycles
    view_layer = bpy.context.view_layer
    saved = {
        "samples": cycles.samples,
        "use_adaptive_sampling": cycles.use_adaptive_sampling,
        "use_denoising": cycles.use_denoising,
        "view_layer_denoising": view_layer.cycles.use_denoising,
        **{field: getattr(cycles, field) for field in _BOUNCE_FIELDS},
    }
    cycles.samples = 1
    cycles.use_adaptive_sampling = False
    cycles.use_denoising = False
    view_layer.cycles.use_denoising = False
    for field in _BOUNCE_FIELDS:
        setattr(cycles, field, 0)
    try:
        yield
    finally:
        cycles.samples = saved["samples"]
        cycles.use_adaptive_sampling = saved["use_adaptive_sampling"]
        cycles.use_denoising = saved["use_denoising"]
        view_layer.cycles.use_denoising = saved["view_layer_denoising"]
        for field in _BOUNCE_FIELDS:
            setattr(cycles, field, saved[field])
