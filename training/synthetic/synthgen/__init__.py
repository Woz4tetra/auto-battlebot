"""Synthetic scene rendering package for YOLO training data generation.

The package is split into two layers:

- Pure modules (importable without Blender): ``constants``, ``configuration``,
  ``logsetup``, ``reporting``, ``gating``, ``annotations``, ``imaging``,
  ``geometry``, ``colorspec``, ``asset_index``.
- Blender-dependent modules (require ``bpy``/``blenderproc``, only importable
  under ``blenderproc run``): ``materials``, ``robots``, ``distractors``,
  ``environment``, ``camera``, ``keypoints``, ``pipeline``.

The entry point is ``training/synthetic/render_scenes.py``, which must import
``blenderproc`` first and defers importing ``synthgen.pipeline`` until runtime.
"""
