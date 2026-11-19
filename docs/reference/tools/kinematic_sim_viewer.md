# Kinematic sim viewer

`simulation/viewer.py` is the in-loop OpenCV viewer for the kinematic sim. It is off by default (`[viewer] enable = false`) so sweeps stay headless.

## Logical time

Rendering costs wall-clock time and cannot corrupt a run. The sim owns logical time and ships `sim_time` in the response header, which the C++ side adopts through `ManualClock`. A slow render makes the sim slower in real time and changes nothing the controller sees. Keep that property when extending the viewer. `test_render_does_not_touch_sim_state` covers it.

## Layers

Drawn cheapest first:

1. Pre-scaled arena background.
2. Hazard discs: the raw geometry plus the inflated keep-out ring the controller steers on.
3. Robot sprites rotated by yaw. A plant that reports `upside_down` is drawn with its `<sprite>_flipped` underside render.
4. The plant's collision outlines.
5. Overlays.

The outlines are window-only, like the overlays. The camera frame is built from `compose_world`, and the detector must not see them.

## Mouse

Press inside an opponent to drag it. Release resumes its configured behaviour from the drop point. Our robot is not draggable: its pose is the plant's integrated state, and moving it out from under the plant desynchronises the EKF in ways that read as control bugs.
