# Workaround ledger

Every workaround the phase 1 comment audit found (2026-11-15, 322 rows), with its root-cause fix. The audit CSV was deleted on 2026-11-20 and is still in git history at `docs/plans/comment_purge_audit.csv`. Fixed rows were deleted as they landed; their commits carry the details.

## Status 2026-11-20

Every row that could be settled on the dev box is gone. It was either deleted, fixed, or pinned by a test that fails with the workaround removed. Checks that need a device are in `docs/plans/hardware_checks.md` ("Ledger rows that need a device"). Three accepted rules stay as comments, listed in `docs/reference/tools/comment_budget.md`.

What is left:

- Two rows deferred to Mk4
- Four rows owned by active plans
- Seven rows kept by decision

## Deferred to Mk4

One root-cause fix retires both: port DShot to the IDF 5 `rmt_tx` driver and pin the IDF 5 platform explicitly.

| Location | What the code does | What breaks without it |
| --- | --- | --- |
| `firmware/espnow_dongle/platformio.ini:8` | Pins `platform = espressif32@6.13.0` (Arduino core on IDF 4.4) | An unpinned `espressif32` resolves to the globally installed pioarduino 55.x (IDF 5), which changes the ESP-NOW frame limits the wire format assumes (250 bytes, `espnow_protocol.h`) |
| `firmware/mr_stabs_mk2/platformio.ini:15` | Pins `platform = espressif32@6.13.0` (Arduino core 2.x, IDF 4.4) | IDF 5 removes NeoPixel's `rmt_reserved_channels` and deprecates the legacy RMT driver `DShot.cpp` uses |

## Owned by a plan

The row closes when the named step lands.

| Location | Plan and step |
| --- | --- |
| `tests/test_kalman_motion_estimator.cpp` (`make_our_pose_measurement`, `make_opponent_pose_measurement`) | `docs/plans/measurement_chain_simplification_plan.md` step 1: delete the blob path, so every measurement carries a yaw and the dummy keypoint goes |
| `include/robot_descriptions_cache.hpp` (`Resolved` lifetime) | `docs/plans/measurement_chain_simplification_plan.md` step 3: `RobotDescriptionsCache` is deleted |
| `src/navigation/rl_policy_navigation.cpp` (coasted spin after the robot is lost) | `docs/plans/measurement_chain_simplification_plan.md` step 3: zero velocity once the mean stops at `max_coast_s`, and tracks expire |
| `include/rl_policy/policy_interface.hpp` (navigation outputs stick values in `VelocityCommand`) | `docs/plans/command_units_plan.md`: navigation outputs real units and `CommandShaper` converts them to sticks |

## Kept by decision

Each workaround stays. Its reason is in the subsystem doc until the follow-up purge removes it.

| Location | Decision |
| --- | --- |
| `include/mask_model/fixed_mask_model.hpp` | Kept 2026-11-20. The kinematic sim's fake 2x2 mask gets the runner past its empty-mask check |
| `hardware/cage_pack/enclosure.py:221` | Kept 2026-11-20. Settling it needs an Onshape push, which costs API calls from the yearly budget |
| `hardware/cage_pack/enclosure.py:295` | Kept 2026-11-20, as above |
| `hardware/zed_box_mini_mount/assembly_page.py:62` | Kept 2026-11-20. Testing GLB handling on the artifact host needs an upload |
| `tests/python/test_tag_pose_smoother.py:578` | Kept 2026-11-20. Retune declined |
| `auto_battlebot/mujoco_sim/onshape_export.py` (wheels-only mass lumping) | Skipped 2026-11-20. Purchased parts have no real mass in the Onshape part studio |
| `training/model_eval/score_size_ladder.sh:97-98` | Skipped 2026-11-20. Needs the `cage_high_x50_conf044/d40000` eval set, which is only on megamind |

## Follow-up: doc purge (done 2026-11-20)

The comment purge had moved code comments into docs. Those are gone:

- 14 docs that held only relocated comments were deleted
- 89 docs had their `Workarounds` sections, line-anchored blocks, test restatements and history removed. Content that says how a subsystem works now stays under plain headings
- 12 dated experiment docs the 2026-11-20 sweep had filed as measurements, but which still carried line anchors, were cleaned the same way
- `GATE_MARGIN`, `TAG_DROPOUT_MIN_GAP_NS` and the other YOLO fix gates moved to `YoloFixSettings`, and `kChannelStreamStallTimeout` to `OpenTxTransmitterConfiguration::channel_stream_stall_timeout_s`
- The rule is in `CLAUDE.md` ("Comments")

Left:

- The sweep's "keep or move" items were not done: how-it-works prose in `docs/experiments/` (the `*_scene_pipeline`, sysid smoother and apriltag docs) still sits there instead of `docs/reference/`, and fitted-value provenance in `camera_calibrations.md`, `robot_plant_files.md` and `synthgen_cage_specs.md` still sits in reference docs
