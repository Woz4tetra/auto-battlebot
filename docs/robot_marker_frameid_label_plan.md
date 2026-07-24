# Plan: unobtrusive frame-id label on robot marker overlay

## Goal

Draw a small, unobtrusive text label next to each robot marker in the UI overlay,
showing the robot's `frame_id` (e.g. `OUR_ROBOT_1`, `THEIR_ROBOT_2`).

## Current behavior

The UI is an LVGL app with an OpenCV-based debug overlay renderer. Robot markers are
pose arrows drawn over the camera image.

- `src/ui/lvgl_platform_bound/lvgl_ui_overlay.cpp`:
  - `draw_robot_pose_arrows(...)` (:53-71) loops over `robots.descriptions`, projects each
    pose to pixels via `project_field_point_to_pixel` (:22-49), and draws:
    ```cpp
    cv::arrowedLine(image, start_px, end_px, to_cv_bgr(robot.group), 2, cv::LINE_AA, 0, 0.25);
    ```
    `start_px` (the projected field-center pixel) is the ideal label anchor.
  - `OpenCvDebugOverlayRenderer::render(...)` (:123-132) orchestrates field border, robot
    arrows, and target path.
- Render is invoked from `src/ui/lvgl_ui.cpp:1031` (`update_debug`).

Each robot already carries its frame id: `RobotDescription.frame_id`
(`include/data_structures/robot.hpp:18-27`, first field). The `FrameId` enum is in
`include/enums/frame_id.hpp:4-17`. String conversion via `magic_enum::enum_name(...)`
(pattern used across the codebase) or `enum_to_string_lower(...)`
(`include/enum_to_string_lower.hpp:8-14`, already transitively included via
`label_utils.hpp`).

There is **no** existing per-overlay-element config toggle. (`local_visualize_debug` in
`config/_common.toml:17` gates a different field-filter debug window, not the UI overlay.)

## Approach

Add the label inside the existing per-robot loop in `draw_robot_pose_arrows`, right after
`start_px` is computed (~:62), matching the codebase's established "unobtrusive text"
style — a two-pass drop-shadow `cv::putText` with no background box. That exact style is
already used for keypoint labels in
`src/keypoint_model/yolo_keypoint_model.cpp:598-605`:

```cpp
const std::string frame_label = std::string(magic_enum::enum_name(robot.frame_id));
const cv::Point label_org(start_px.x + 8, start_px.y - 8);   // offset off the arrow origin
cv::putText(image, frame_label, label_org + cv::Point(1, 1),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);   // shadow
cv::putText(image, frame_label, label_org,
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA); // text
```

Scale 0.5, white fill on a black outline reads clearly over the camera image without a
heavy filled box — the "unobtrusive" requirement.

### Steps

1. **Add the label draw** in `draw_robot_pose_arrows`
   (`src/ui/lvgl_platform_bound/lvgl_ui_overlay.cpp:55-70`), after `start_px`.
2. **Include** `magic_enum.hpp` if not already present (or use `enum_to_string_lower`,
   which is already available). The overlay already includes `label_utils.hpp` and
   OpenCV `imgproc`, so no new build dependencies.
3. **Skip empties**: don't draw when `robot.frame_id == FrameId::EMPTY`.
4. **Optional color match**: use `get_color_for_index(robot.frame_id)`
   (`include/label_utils.hpp:79`, `ColorRGBf::to_bgr_255()`) for the fill color so the
   label matches the marker's identity color instead of plain white — decide during
   review.
5. **Optional config toggle**: if the label should be switchable, add a
   `show_frame_id_labels` bool to the UI config (parse pattern `PARSE_FIELD_BOOL`, config
   path around `lvgl_ui.cpp` / `lvgl_ui_platform.cpp`) and gate the draw. Default on.

## Design decisions

- **Label text**: full enum name (`OUR_ROBOT_1`) vs. short form via
  `get_short_name(...)` (`include/label_utils.hpp:99-105`, yields `"1"`). Full name is
  more informative and still short; recommend full name at scale 0.5. Confirm preference.
- **Anchor/offset**: place above-left of `start_px` so it doesn't overlap the arrow
  shaft. Tune the offset so labels of nearby robots don't collide (the yolo box-label
  code at `:545-582` has overlap-avoidance logic to borrow if collisions are a problem).

## Testing

- Build and run playback: `./scripts/build_and_run.sh -c config/playback.toml`, confirm
  each robot marker shows its frame id, legible over varied backgrounds, no overlap with
  the arrow.
- Verify empty/stale robots don't render stray labels.

## Validation

```bash
./scripts/build.sh
git diff --name-only HEAD | grep '\.cpp$' | xargs -r clang-tidy -p build-test/
./scripts/lint
```

This is a small, contained change — one draw block in one function.
