# Simulation

Physics-accurate simulation of Mr. Stabs Mk2 in the NHRL combat arena, backed by [MuJoCo](https://mujoco.org/). Used for perception regression testing and autonomy development without hardware.

## Quick start

```bash
./scripts/run_simulation.sh                         # default config
./scripts/run_simulation.sh simulation/sim_config.toml config/simulation.toml
```

The script builds the C++ binary and starts the Python sim server in parallel. Press **Ctrl-C** to stop both.

To run the sim server standalone (e.g. to inspect rendering without the C++ pipeline):

```bash
cd simulation
source venv/bin/activate
python sim_server.py sim_config.toml
```

## Architecture

```
C++ pipeline (Runner / SimRgbdCamera)
        │  TCP 127.0.0.1:14882
        │  ┌─ request: VelocityCommand (3 × float64)
        │  └─ response: header + RGB + depth + ground-truth poses
        ▼
Python sim server (sim_server.py)
  ├── scene.py       — MuJoCo model assembly, mesh pre-processing
  ├── runner.py      — physics step loop, renderer, socket I/O
  ├── behaviors.py   — opponent AI strategies
  └── protocol.py    — wire-format constants (kept in sync with sim_connection.hpp)
```

Each frame:
1. C++ sends a `VelocityCommand` (`linear_x`, `linear_y`, `angular_z` in [-1, 1]).
2. The server converts it to per-wheel velocity targets using differential-drive kinematics, applies MuJoCo velocity actuators, steps physics (`substeps` × `mj_step`), and renders.
3. The server sends back: a response header (image dimensions, camera intrinsics, camera→world transform), the RGB frame (BGR, `res_width × res_height × 3` uint8), the depth frame (`float32`, metres, NaN for background), and ground-truth poses (x, y, yaw in metres/radians) for all robots.

### Physics

MuJoCo SAP/CG solver. Each wheel joint has:

| MJCF attribute | Meaning |
|---|---|
| `armature` | Reflected rotor inertia — controls acceleration rate |
| `damping` | Back-EMF viscous damping — controls deceleration |
| `frictionloss` | Gearbox Coulomb stiction |
| Velocity actuator `kv` | Proportional torque per rad/s error |
| Contact `friction` (3 values) | Longitudinal, lateral, rolling |

Wheel contact geometry is a cylinder; the chassis collision shape is a simplified convex hull (`chassis_collision.obj`). Casters are tiny spheres (radius 1 mm) with near-zero friction.

### Rendering

`mujoco.Renderer` with EGL offscreen backend. Depth is in metres, directly usable as `float32`. Far-plane pixels are set to NaN. If a panorama is configured it is composited behind the rendered scene at startup using equirectangular projection.

### Mesh pre-processing

trimesh-exported OBJ files use a non-standard interleaved per-material format that MuJoCo's loader cannot fully read. `scene.py` pre-processes them at first startup:

- `_split_obj_by_material()` — exports one OBJ per material group into `assets/robots/<robot>/meshes/_mujoco/`. Subsequent runs use the cache (invalidated if the source OBJ is newer).
- `_convert_glb()` — converts GLB files to per-geometry OBJs in `assets/robots/_mujoco/`. Geometry keys encode solid RGB colours (`mat_R_G_B`) which are decoded to MJCF `rgba` attributes.

Collision geoms are placed in MuJoCo group 3 (hidden by the renderer's default `MjvOption`); only visual geoms in group 0 are rendered.

## Configuration (`sim_config.toml`)

### `[server]`

| Key | Default | Meaning |
|---|---|---|
| `physics_dt` | 0.01 s | Wall-clock target time per physics frame |
| `substeps` | 10 | `mj_step` calls per frame (effective timestep 1 ms) |
| `settle_steps` | 50 | Steps run at startup before serving to let robots settle |
| `max_physics_steps_per_frame` | 10 | Cap on physics steps when the server falls behind |

### `[arena]`

| Key | Meaning |
|---|---|
| `width`, `height` | Arena floor dimensions in metres. NHRL cage is 2.4 × 2.4 m. |
| `floor_friction` | Tangential contact friction coefficient for the floor |
| `panorama` | Filename from `assets/panoramas/` composited as the background (omit for plain) |

### `[camera]`

Matches the ZED 2i mounted overhead. `fov` is vertical FOV in degrees. `pos` and `lookat` define the camera pose in world space (metres, Z-up).

### `[our_robot]` / `[[opponents]]`

| Key | Meaning |
|---|---|
| `model_path` | URDF or GLB relative to the config file |
| `start_pos` | Spawn position [x, y] or [x, y, z] in metres |
| `start_rotation` | Yaw at spawn in degrees |
| `model_euler` | Euler angles (XYZ, degrees) to correct model orientation from URDF export |
| `max_linear_speed` | Maps command=1.0 to this m/s — a software limit, not the physical maximum |
| `max_angular_speed` | Maps command=1.0 to this rad/s |
| `behavior` | Opponent AI: `static`, `random_walk`, or `circular` |
| `speed` | Movement speed for `random_walk` / `circular` opponents (m/s) |

Wheel-drive parameters (URDF robots only):

| Key | Meaning |
|---|---|
| `wheel_radius` | Metres |
| `track_width` | Centre-to-centre wheel separation, metres |
| `left_wheel_joints` / `right_wheel_joints` | Joint names in the URDF |
| `wheel_kv` | Velocity actuator proportional gain (N·m per rad/s error) |
| `wheel_force_limit` | Maximum actuator force (N) |
| `wheel_armature` | Reflected rotor inertia (kg·m²) |
| `wheel_damping` | Viscous damping (N·m·s/rad) |
| `wheel_frictionloss` | Coulomb stiction (N·m) |
| `wheel_contact_friction` | `[longitudinal, lateral, rolling]` contact friction |

### Opponent behaviors

- **`static`** — stays at spawn position.
- **`random_walk`** — picks random targets within 80 % of the arena bounds and drives toward them.
- **`circular`** — orbits a fixed centre point. Physics-simulated robots use velocity actuators; kinematic (GLB) robots are moved directly via `mocap_pos`.

## Robot parameters

### Mr. Stabs Mk2 (our robot)

Source: URDF exported from OnShape by the CAD model.

| Parameter | Value | Source |
|---|---|---|
| Chassis mass | 422.8 g | URDF `<inertial>` |
| Wheel radius | 25 mm | URDF cylinder collision |
| Track width | 128 mm | URDF wheel joint origins (y = ±64 mm) |
| Motor | Repeat Robotics Mk4 Mini 1104, 3500 kV | Hardware |
| Gearbox | 28.5:1 planetary | Hardware |
| Battery | 2× 2S HV LiPo in series (4S HV), nominal 15.2 V | Hardware |
| ESC current limit | 25 A | Hardware |

Derived values:

| Value | Calculation |
|---|---|
| Max wheel speed | 3500 kV × 15.2 V / 28.5 = 1867 RPM → 195 rad/s → **4.89 m/s** linear |
| `wheel_armature` 1.2×10⁻⁴ kg·m² | Rotor I ≈ 1.5×10⁻⁷ kg·m²; reflected: × 28.5² = 812 |
| `wheel_damping` 0.034 N·m·s/rad | Back-EMF: Kₑ²/R × gear² × η = (Kₑ = 1/kV in SI) |
| `wheel_force_limit` 80 N | 25 A × Kₜ × 28.5 × 0.85 / wheel_radius |

`max_linear_speed` and `max_angular_speed` in the config are software limits used to scale commands — they are set conservatively below the physical maximum for stable autonomy behaviour.

### house_bot (opponent)

A generic wheeled opponent (4-wheel differential drive). The mass and geometry come from the URDF. Physics parameters use the same MJCF structure as Mr. Stabs but with generic values (no matched motor spec). Set to `behavior = "static"` by default.

### Mrs. Buff Mk2 (opponent)

A kinematically-controlled opponent (no physics applied to its body). The GLB model contains solid-colour geometry only (no textures); colours are decoded from the geometry names (`mat_R_G_B` → MJCF `rgba`). Set to `behavior = "circular"` by default. To change its movement pattern, set `behavior` and `speed` in the config.

## Assets

```
simulation/assets/
├── robots/
│   ├── mr_stabs_mk2/           URDF + OBJ meshes with PBR textures
│   │   └── meshes/_mujoco/     Auto-generated per-material OBJ cache (git-ignored)
│   ├── house_bot/              URDF + OBJ meshes
│   │   └── meshes/_mujoco/     Auto-generated cache
│   ├── mrs_buff_mk2.glb        Solid-colour GLB model
│   └── _mujoco/                Auto-generated GLB→OBJ cache (git-ignored)
└── panoramas/                  Equirectangular background images
```

The `_mujoco/` cache directories are created automatically on first run. Delete them to force a rebuild.

## Dependencies

Python 3.12, managed in `simulation/venv/`. Key packages:

| Package | Role |
|---|---|
| `mujoco >= 3.1` | Physics and rendering |
| `trimesh` | Mesh pre-processing (OBJ splitting, GLB conversion) |
| `opencv-python` | Image colour conversion |
| `scipy` | Quaternion composition for robot spawn poses |
| `numpy` | Array operations |
| `dacite` | TOML → typed dataclass loading |

Install / update the venv:

```bash
cd simulation
python3 -m venv venv
venv/bin/pip install -r requirements.txt
```
