"""Tests for the typed config layer against the real config.toml and edge cases."""

from pathlib import Path

import numpy as np
import pytest
from synthgen.configuration import (
    CageConfig,
    ConfigError,
    PathResolver,
    _parse_cages,
    apply_damage_mode,
    apply_damage_parts,
    apply_venue,
    apply_view,
    choose_cage,
    load_render_config,
)

REAL_CONFIG = Path(__file__).resolve().parents[1] / "config.toml"

MINIMAL_TOML = """
[output]
image_dir = "images"
label_dir = "labels"
num_images = 10

[[robots]]
model_path = "robot.gltf"
class_id = 0

[robots.keypoints]
front = [0.1, 0.0, 0.0]
back = [-0.1, 0.0, 0.0]

[[robots.color_mapping]]
color = [128, 128, 128]
tolerance = 1
material = "aluminum"
"""


def _write_config(tmp_path: Path, content: str) -> Path:
    config_path = tmp_path / "config.toml"
    config_path.write_text(content)
    return config_path


class TestRealConfig:
    def test_parses_shipped_config(self) -> None:
        cfg = load_render_config(REAL_CONFIG)

        assert cfg.output.num_images == 20000
        assert cfg.output.image_width == 1280
        assert cfg.output.image_height == 720
        assert cfg.output.images_per_scene == 10
        assert cfg.output.min_robot_visibility == pytest.approx(0.10)
        assert cfg.output.ignore_obstructions is False
        assert cfg.output.memory_cleanup_interval == 1000
        assert cfg.output.annotation_mode == "keypoints_bbox"
        assert cfg.output.is_segmentation_mode is False

        assert len(cfg.robots) == 2
        stabs, buff = cfg.robots
        assert stabs.name == "mr_stabs_mk2"
        assert stabs.class_id == 0
        assert stabs.weight == pytest.approx(0.5)
        assert stabs.ground_roll_upright == pytest.approx(9.587585)
        # Keypoints are converted to Blender axes: (x, y, z) -> (x, -z, y).
        np.testing.assert_allclose(stabs.keypoints.front, [0.1115, 0.00313, 0.0])
        assert buff.name == "mrs_buff_mk3"
        assert buff.class_id == 1
        assert buff.ground_roll_inverted == pytest.approx(-30.0)
        assert len(buff.color_mapping) > len(stabs.color_mapping)

        assert cfg.materials["aluminum"].metallic == pytest.approx(1.0)
        assert cfg.materials["aluminum"].cc_texture == "Metal012"
        assert cfg.materials["housebot"].texture_dir is not None

        assert cfg.distractors.min_per_scene == 1
        assert cfg.distractors.max_per_scene == 5
        assert cfg.distractors.vram_budget_mb == pytest.approx(3500.0)
        assert cfg.distractors.base_dimension_m == pytest.approx(0.25)
        assert cfg.distractors.scale_range == (0.5, 2.0)
        assert cfg.distractors.robot_air_probability == pytest.approx(0.1)
        assert cfg.distractors.has_cad_source() is True
        kinds = [s.effective_kind() for s in cfg.distractors.sources]
        assert "cad" in kinds

        assert cfg.camera.max_distance == pytest.approx(6.0)
        assert cfg.camera.max_frame_fraction == pytest.approx(0.85)
        assert cfg.scene.max_robots_per_scene == 10
        assert cfg.randomization.air_probability == pytest.approx(0.3)
        assert cfg.randomization.motion_blur_strength_range == (5, 15)

        assert [cage.name for cage in cfg.cages] == ["nhrl_cage", "massd_arena"]
        nhrl, massd = cfg.cages
        assert nhrl.enabled is True
        assert nhrl.probability == pytest.approx(0.5)
        # 64 with the spec's OptiX denoiser matched 128 with the compositor denoiser on
        # 2026-09-12 (37.6 vs 39.9 dB PSNR against a 512-sample reference, crops alike).
        assert nhrl.render_samples == 64
        assert massd.render_samples == 64
        assert nhrl.mount.walls == ("near", "far", "left", "right")
        assert nhrl.mount.height_m == (0.60, 1.25)
        assert massd.probability == pytest.approx(0.25)
        # Both mounts stand outside the wall, close to it, with tilt derived from where they
        # land and aimed short of the field centre. NHRL's come from the 12 poses flown in
        # pose_camera_server.py on 2026-09-12; MassD's come from the coverage sweeps, pulled
        # in once the half moved to the wider e-CAM25 lens.
        for mount in (nhrl.mount, massd.mount):
            assert mount.aim == "centre"
            # Strictly outside, so the pane the camera looks through is always hidden.
            assert mount.inset_m[0] < 0.0 and mount.inset_m[1] < 0.0
            # Aimed short of the centre, never past it.
            assert mount.tilt_offset_deg[0] < 0.0 and mount.tilt_offset_deg[1] < 0.0
        assert nhrl.mount.inset_m == (-0.20, -0.08)
        assert nhrl.mount.tilt_offset_deg == (-22.0, -10.0)
        assert massd.mount.inset_m == (-0.25, -0.10)
        assert massd.mount.height_m == (0.45, 1.10)
        # Both mounts stay clear of the glass. A camera within 0.055 m of the wall put an
        # adjacent pane across a 118 degree frame edge-on, and the rough refraction cost 402
        # seconds per render pass at 32 samples where a healthy scene takes 4 to 23 at 128.
        for mount in (nhrl.mount, massd.mount):
            assert mount.inset_m[1] <= -0.08
        # Both halves stand in for our own camera, so both render through its lens.
        assert nhrl.camera_calibration == massd.camera_calibration
        # Every spec and camera calibration resolves against the config directory.
        for cage in cfg.cages:
            assert cfg.resolver.resolve(cage.spec).exists()
            assert cfg.resolver.resolve(cage.camera_calibration).exists()

    def test_venue_flag_pins_one_venue_or_the_arena(self) -> None:
        cfg = load_render_config(REAL_CONFIG)

        nhrl = apply_venue(cfg, "nhrl_cage")
        assert [cage.name for cage in nhrl.cages if cage.active] == ["nhrl_cage"]
        assert [cage.probability for cage in nhrl.cages if cage.active] == [1.0]

        assert apply_venue(cfg, "arena").cages == ()
        assert apply_venue(cfg, None) is cfg
        with pytest.raises(ConfigError, match="only_cage"):
            apply_venue(cfg, "no_such_cage")

    def test_damage_flag_overrides_the_split(self) -> None:
        cfg = load_render_config(REAL_CONFIG)
        assert cfg.damage.enabled is True
        assert cfg.damage.scene_probability == pytest.approx(0.5)

        assert apply_damage_mode(cfg, "config") is cfg
        assert apply_damage_mode(cfg, "off").damage.enabled is False
        every = apply_damage_mode(cfg, "all").damage
        assert every.enabled is True
        assert every.scene_probability == pytest.approx(1.0)
        assert every.probability == pytest.approx(1.0)
        # Everything else about the draw is the config's.
        assert every.part_severity == cfg.damage.part_severity
        with pytest.raises(ConfigError, match="damage mode"):
            apply_damage_mode(cfg, "sometimes")

    def test_cage_split_tracks_every_ratio_at_once(self) -> None:
        cages = [
            CageConfig(name="half", enabled=True, probability=0.5),
            CageConfig(name="quarter", enabled=True, probability=0.25),
        ]
        counts = [0, 0]
        for written in range(200):
            chosen = choose_cage(cages, written, counts)
            if chosen is not None:
                counts[chosen] += 1
        # The leftover is the HDRI arena's share, and no cage steals from the other.
        assert counts == [100, 50]

    def test_cage_split_is_off_when_disabled_or_zero(self) -> None:
        assert choose_cage([CageConfig(enabled=False, probability=1.0)], 0, [0]) is None
        assert choose_cage([CageConfig(enabled=True, probability=0.0)], 0, [0]) is None
        assert choose_cage([CageConfig(enabled=True, probability=1.0)], 10, [10]) == 0

    def test_cage_probabilities_may_not_exceed_the_whole_run(self) -> None:
        raw = {
            "cages": [
                {"name": "a", "enabled": True, "probability": 0.8},
                {"name": "b", "enabled": True, "probability": 0.5},
            ]
        }
        with pytest.raises(ConfigError, match="sum to 1.30"):
            _parse_cages(raw["cages"])

    def test_cage_names_must_be_unique(self) -> None:
        with pytest.raises(ConfigError, match="unique"):
            _parse_cages([{"name": "a"}, {"name": "a"}])

    def test_resolver_points_at_config_dir(self) -> None:
        cfg = load_render_config(REAL_CONFIG)
        assert cfg.resolver.config_dir == REAL_CONFIG.parent


class TestDefaults:
    def test_minimal_config_gets_code_defaults(self, tmp_path: Path) -> None:
        cfg = load_render_config(_write_config(tmp_path, MINIMAL_TOML))

        assert cfg.output.annotation_mode == "keypoints_bbox"
        assert cfg.output.image_width == 1280
        assert cfg.output.images_per_scene == 5
        assert cfg.output.min_robot_visibility == pytest.approx(0.10)
        assert cfg.output.ignore_obstructions is False
        assert cfg.output.memory_cleanup_interval == 25
        assert cfg.output.segmentation_min_bbox_dim == 1

        robot = cfg.robots[0]
        assert robot.name == "robot"
        assert robot.scale == pytest.approx(1.0)
        assert robot.weight == pytest.approx(1.0)
        assert robot.ground_roll_upright == pytest.approx(0.0)

        assert cfg.distractors.sources == ()
        assert cfg.distractors.min_per_scene == 0
        assert cfg.distractors.max_per_scene == 5
        assert cfg.distractors.vram_budget_mb is None
        assert cfg.distractors.scale_range == (0.5, 3.0)
        assert cfg.distractors.robot_air_probability is None
        assert cfg.distractors.has_cad_source() is False

        assert cfg.camera.min_distance == pytest.approx(0.3)
        assert cfg.camera.max_distance == pytest.approx(1.5)
        assert cfg.camera.height_range == (0.1, 0.8)
        assert cfg.camera.look_at_noise == pytest.approx(0.05)
        assert cfg.camera.max_frame_fraction == pytest.approx(0.9)

        assert cfg.scene.ground_size_range == (2.0, 5.0)
        assert cfg.scene.arena_radius_range == (0.5, 1.5)
        assert cfg.scene.ground_visibility == pytest.approx(0.8)
        assert cfg.scene.max_robots_per_scene == 1

        assert cfg.randomization.air_probability == pytest.approx(0.15)
        assert cfg.randomization.air_height_range == (0.02, 0.15)
        assert cfg.randomization.motion_blur_probability == pytest.approx(0.0)
        assert cfg.randomization.motion_blur_strength_range == (5, 25)
        assert cfg.randomization.light_count_range == (1, 3)

    def test_unknown_keys_tolerated(self, tmp_path: Path) -> None:
        toml = MINIMAL_TOML + "\n".join(
            [
                "",
                "[randomization]",
                "light_color_temp_range = [3500, 6500]",
                "",
                "[environment]",
                'ground_textures = ["Road013C"]',
                "",
                "[unknown_section]",
                "whatever = 1",
            ]
        )
        cfg = load_render_config(_write_config(tmp_path, toml))
        assert cfg.randomization.light_count_range == (1, 3)


class TestErrors:
    def test_missing_config_file(self, tmp_path: Path) -> None:
        with pytest.raises(ConfigError, match="not found"):
            load_render_config(tmp_path / "nope.toml")

    def test_missing_output_section(self, tmp_path: Path) -> None:
        with pytest.raises(ConfigError, match=r"\[output\]"):
            load_render_config(_write_config(tmp_path, "[scene]\n"))

    def test_missing_robots(self, tmp_path: Path) -> None:
        toml = '[output]\nimage_dir = "i"\nlabel_dir = "l"\nnum_images = 1\n'
        with pytest.raises(ConfigError, match=r"\[\[robots\]\]"):
            load_render_config(_write_config(tmp_path, toml))

    def test_missing_required_output_key(self, tmp_path: Path) -> None:
        toml = MINIMAL_TOML.replace("num_images = 10\n", "")
        with pytest.raises(ConfigError, match="num_images"):
            load_render_config(_write_config(tmp_path, toml))

    def test_missing_robot_keypoints(self, tmp_path: Path) -> None:
        toml = MINIMAL_TOML.replace("[robots.keypoints]", "[robots.keypoints_typo]").replace(
            "front = [0.1, 0.0, 0.0]", "front_typo = [0.1, 0.0, 0.0]"
        )
        with pytest.raises(ConfigError, match="keypoints"):
            load_render_config(_write_config(tmp_path, toml))

    def test_invalid_annotation_mode(self, tmp_path: Path) -> None:
        toml = MINIMAL_TOML.replace("num_images = 10", 'num_images = 10\nannotation_mode = "bogus"')
        with pytest.raises(ConfigError, match="annotation_mode"):
            load_render_config(_write_config(tmp_path, toml))

    def test_malformed_number(self, tmp_path: Path) -> None:
        toml = MINIMAL_TOML.replace("num_images = 10", 'num_images = "ten"')
        with pytest.raises(ConfigError, match="num_images"):
            load_render_config(_write_config(tmp_path, toml))


class TestPathResolver:
    def test_absolute_path_passes_through(self, tmp_path: Path) -> None:
        resolver = PathResolver(tmp_path, tmp_path, tmp_path)
        assert resolver.resolve(Path("/etc/hosts")) == Path("/etc/hosts")

    def test_candidate_order(self, tmp_path: Path) -> None:
        config_dir = tmp_path / "config"
        launch_cwd = tmp_path / "cwd"
        project_root = tmp_path / "root"
        for d in (config_dir, launch_cwd, project_root):
            (d / "asset").mkdir(parents=True)

        resolver = PathResolver(config_dir, launch_cwd, project_root)
        assert resolver.resolve(Path("asset")) == (config_dir / "asset").resolve()

        # Config-dir candidate missing: falls through to launch CWD.
        resolver2 = PathResolver(tmp_path / "missing", launch_cwd, project_root)
        assert resolver2.resolve(Path("asset")) == (launch_cwd / "asset").resolve()

    def test_nonexistent_returns_first_candidate(self, tmp_path: Path) -> None:
        resolver = PathResolver(tmp_path / "a", tmp_path / "b", tmp_path / "c")
        assert resolver.resolve(Path("ghost")) == (tmp_path / "a" / "ghost").resolve()


class TestExtends:
    """A per-venue config says what differs and inherits the rest."""

    def test_child_inherits_and_overrides(self, tmp_path: Path) -> None:
        _write_config(tmp_path, MINIMAL_TOML)
        child = tmp_path / "variant.toml"
        child.write_text('extends = "config.toml"\n\n[output]\nnum_images = 250\n')
        cfg = load_render_config(child)
        # Overridden key takes the child's value, the rest of [output] survives the merge.
        assert cfg.output.num_images == 250
        assert cfg.output.image_dir.name == "images"
        assert len(cfg.robots) == 1

    def test_arrays_replace_rather_than_merge(self, tmp_path: Path) -> None:
        _write_config(tmp_path, MINIMAL_TOML + '\n[[cages]]\nname = "a"\nprobability = 0.5\n')
        child = tmp_path / "variant.toml"
        child.write_text('extends = "config.toml"\n[[cages]]\nname = "b"\nprobability = 0.25\n')
        cfg = load_render_config(child)
        assert [cage.name for cage in cfg.cages] == ["b"]

    def test_cycle_is_an_error(self, tmp_path: Path) -> None:
        (tmp_path / "a.toml").write_text('extends = "b.toml"\n')
        (tmp_path / "b.toml").write_text('extends = "a.toml"\n')
        with pytest.raises(ConfigError, match="extends cycle"):
            load_render_config(tmp_path / "a.toml")

    def test_missing_parent_is_an_error(self, tmp_path: Path) -> None:
        child = tmp_path / "variant.toml"
        child.write_text('extends = "nope.toml"\n')
        with pytest.raises(ConfigError, match="extends target not found"):
            load_render_config(child)

    def test_parent_must_be_a_sibling(self, tmp_path: Path) -> None:
        child = tmp_path / "variant.toml"
        child.write_text('extends = "sub/config.toml"\n')
        with pytest.raises(ConfigError, match="same directory"):
            load_render_config(child)

    def test_shipped_venue_configs_pin_one_cage_each(self) -> None:
        for name, wanted in (
            ("config_cage_nhrl.toml", "nhrl_cage"),
            ("config_cage_massd.toml", "massd_arena"),
        ):
            cfg = load_render_config(REAL_CONFIG.parent / name)
            active = [cage for cage in cfg.cages if cage.active]
            assert [cage.name for cage in active] == [wanted]
            assert active[0].probability == pytest.approx(1.0)
            # Inherited from config.toml rather than restated.
            assert [robot.name for robot in cfg.robots] == ["mr_stabs_mk2", "mrs_buff_mk3"]
            assert cfg.damage.enabled is True


class TestOnlyCage:
    def test_unknown_name_is_an_error(self, tmp_path: Path) -> None:
        # only_cage is a top-level key, so it goes before the first table header.
        toml = 'only_cage = "nope"\n' + MINIMAL_TOML + '\n[[cages]]\nname = "a"\n'
        with pytest.raises(ConfigError, match="no \\[\\[cages\\]\\] entry"):
            load_render_config(_write_config(tmp_path, toml))

    def test_other_cages_are_disabled(self, tmp_path: Path) -> None:
        toml = (
            'only_cage = "b"\n'
            + MINIMAL_TOML
            + '\n[[cages]]\nname = "a"\nenabled = true\nprobability = 0.5\n'
            + '\n[[cages]]\nname = "b"\nenabled = false\nprobability = 0.25\n'
        )
        cfg = load_render_config(_write_config(tmp_path, toml))
        by_name = {cage.name: cage for cage in cfg.cages}
        assert by_name["b"].enabled and by_name["b"].probability == pytest.approx(1.0)
        assert not by_name["a"].enabled


class TestDamage:
    def test_defaults_are_off(self, tmp_path: Path) -> None:
        cfg = load_render_config(_write_config(tmp_path, MINIMAL_TOML))
        assert cfg.damage.enabled is False

    def test_shipped_config_enables_damage(self) -> None:
        damage = load_render_config(REAL_CONFIG).damage
        assert damage.enabled is True
        assert damage.probability == pytest.approx(0.35)
        # The clean pool a damage-off arm draws from is 1 - scene_probability.
        assert damage.scene_probability == pytest.approx(0.5)
        assert damage.part_severity == (0.05, 0.30)
        assert damage.chunk_volume_fraction == (0.03, 0.20)
        assert damage.cutter_pool_size >= 1

    def test_unknown_cutter_shape_is_an_error(self, tmp_path: Path) -> None:
        toml = MINIMAL_TOML + '\n[damage]\ncutter_shapes = ["torus"]\n'
        with pytest.raises(ConfigError, match="cutter_shapes"):
            load_render_config(_write_config(tmp_path, toml))

    def test_name_patterns_are_lowercased(self, tmp_path: Path) -> None:
        toml = MINIMAL_TOML + '\n[damage]\nprotected_name_patterns = ["Chassis"]\n'
        cfg = load_render_config(_write_config(tmp_path, toml))
        assert cfg.damage.protected_name_patterns == ("chassis",)


PART_TOML = """
[[robots.damage_parts]]
name = "disk"
boxes = [[0.0, 0.0, 0.0, 0.1, 0.1, 0.1]]

[[robots.damage_parts]]
name = "module"
includes = ["disk"]
boxes = [[0.0, 0.0, 0.0, 0.2, 0.2, 0.2]]
"""


class TestDamageParts:
    def test_shipped_config_names_mrs_buff_parts(self) -> None:
        cfg = load_render_config(REAL_CONFIG)
        buff = next(robot for robot in cfg.robots if robot.name == "mrs_buff_mk3")
        parts = {part.name: part for part in buff.damage_parts}
        assert list(parts) == [
            "top_sticker",
            "bottom_sticker",
            "weapon_disk",
            "wheels",
            "wheel_side_guards",
            "weapon_module",
            "top_plate",
            "bottom_plate",
        ]
        assert parts["wheels"].subset and len(parts["wheels"].pieces) == 4
        # A guard is one printed piece per side, a side wall and a front arm.
        guards = parts["wheel_side_guards"]
        assert guards.subset and [len(piece) for piece in guards.pieces] == [2, 2]
        # One wheel or one guard at a time is the usual loss.
        assert parts["wheels"].count_weights[0] == max(parts["wheels"].count_weights)
        assert guards.count_weights[0] > guards.count_weights[1]
        assert parts["weapon_module"].includes == ("weapon_disk",)
        assert parts["top_plate"].includes == ("top_sticker",)
        assert not parts["top_sticker"].selectable
        assert cfg.damage.removable_parts == ()
        assert cfg.damage.part_count == (1, 3)

    def test_parses_parts(self, tmp_path: Path) -> None:
        cfg = load_render_config(_write_config(tmp_path, MINIMAL_TOML + PART_TOML))
        disk, module = cfg.robots[0].damage_parts
        assert disk.pieces == (((0.0, 0.0, 0.0, 0.1, 0.1, 0.1),),)
        assert module.includes == ("disk",) and not module.subset

    def test_unknown_include_is_an_error(self, tmp_path: Path) -> None:
        toml = MINIMAL_TOML + PART_TOML.replace('includes = ["disk"]', 'includes = ["gone"]')
        with pytest.raises(ConfigError, match="includes"):
            load_render_config(_write_config(tmp_path, toml))

    def test_inverted_box_is_an_error(self, tmp_path: Path) -> None:
        toml = MINIMAL_TOML + PART_TOML.replace("0.1, 0.1, 0.1]", "-0.1, 0.1, 0.1]")
        with pytest.raises(ConfigError, match="min bound"):
            load_render_config(_write_config(tmp_path, toml))

    def test_unknown_removable_part_is_an_error(self, tmp_path: Path) -> None:
        toml = MINIMAL_TOML + PART_TOML + '\n[damage]\nremovable_parts = ["wings"]\n'
        with pytest.raises(ConfigError, match="wings"):
            load_render_config(_write_config(tmp_path, toml))

    def test_part_count_must_start_at_one(self, tmp_path: Path) -> None:
        toml = MINIMAL_TOML + "\n[damage]\npart_count = [0, 2]\n"
        with pytest.raises(ConfigError, match="part_count"):
            load_render_config(_write_config(tmp_path, toml))

    def test_cli_override_pins_the_batch(self, tmp_path: Path) -> None:
        cfg = load_render_config(_write_config(tmp_path, MINIMAL_TOML + PART_TOML))
        assert apply_damage_parts(cfg, None) is cfg
        assert apply_damage_parts(cfg, ["module"]).damage.removable_parts == ("module",)
        with pytest.raises(ConfigError, match="--damage-parts"):
            apply_damage_parts(cfg, ["wings"])

    def test_count_weights_need_a_subset_part(self, tmp_path: Path) -> None:
        toml = MINIMAL_TOML + PART_TOML.replace(
            'name = "disk"', 'name = "disk"\ncount_weights = [1.0]'
        )
        with pytest.raises(ConfigError, match="count_weights"):
            load_render_config(_write_config(tmp_path, toml))

    def test_unselectable_part_is_not_a_valid_batch_choice(self, tmp_path: Path) -> None:
        toml = MINIMAL_TOML + PART_TOML.replace(
            'name = "disk"', 'name = "disk"\nselectable = false'
        )
        cfg = load_render_config(_write_config(tmp_path, toml))
        with pytest.raises(ConfigError, match="disk"):
            apply_damage_parts(cfg, ["disk"])

    def test_pieces_group_boxes(self, tmp_path: Path) -> None:
        toml = (
            MINIMAL_TOML
            + """
[[robots.damage_parts]]
name = "guards"
subset = true
count_weights = [3.0, 1.0]
pieces = [
  [[0.0, 0.0, 0.0, 0.1, 0.1, 0.1], [0.2, 0.2, 0.2, 0.3, 0.3, 0.3]],
  [[-0.1, -0.1, -0.1, 0.0, 0.0, 0.0]],
]
"""
        )
        (guards,) = load_render_config(_write_config(tmp_path, toml)).robots[0].damage_parts
        assert [len(piece) for piece in guards.pieces] == [2, 1]
        assert guards.count_weights == (3.0, 1.0)

    def test_boxes_and_pieces_are_exclusive(self, tmp_path: Path) -> None:
        toml = MINIMAL_TOML + PART_TOML.replace(
            'name = "disk"', 'name = "disk"\nsubset = true\npieces = [[[0, 0, 0, 1, 1, 1]]]'
        )
        with pytest.raises(ConfigError, match="exactly one"):
            load_render_config(_write_config(tmp_path, toml))

    def test_pieces_need_a_subset_part(self, tmp_path: Path) -> None:
        toml = (
            MINIMAL_TOML
            + """
[[robots.damage_parts]]
name = "guards"
pieces = [[[0.0, 0.0, 0.0, 0.1, 0.1, 0.1]]]
"""
        )
        with pytest.raises(ConfigError, match="subset"):
            load_render_config(_write_config(tmp_path, toml))


class TestCageViews:
    def test_default_view_is_pinhole_at_alpha_one(self, tmp_path: Path) -> None:
        toml = MINIMAL_TOML + '\n[[cages]]\nname = "a"\n'
        (cage,) = load_render_config(_write_config(tmp_path, toml)).cages
        assert cage.view == "pinhole" and cage.rectify_alpha == 1.0

    def test_parses_view_and_alpha(self, tmp_path: Path) -> None:
        toml = MINIMAL_TOML + '\n[[cages]]\nname = "a"\nview = "distorted"\nrectify_alpha = 0.0\n'
        (cage,) = load_render_config(_write_config(tmp_path, toml)).cages
        assert cage.view == "distorted" and cage.rectify_alpha == 0.0

    def test_unknown_view_is_an_error(self, tmp_path: Path) -> None:
        toml = MINIMAL_TOML + '\n[[cages]]\nname = "a"\nview = "fisheye"\n'
        with pytest.raises(ConfigError, match="view"):
            load_render_config(_write_config(tmp_path, toml))

    def test_alpha_outside_zero_to_one_is_an_error(self, tmp_path: Path) -> None:
        toml = MINIMAL_TOML + '\n[[cages]]\nname = "a"\nrectify_alpha = 1.5\n'
        with pytest.raises(ConfigError, match="rectify_alpha"):
            load_render_config(_write_config(tmp_path, toml))

    def test_cli_view_applies_to_every_cage(self, tmp_path: Path) -> None:
        toml = (
            MINIMAL_TOML + '\n[[cages]]\nname = "a"\n\n[[cages]]\nname = "b"\nview = "rectified"\n'
        )
        cfg = load_render_config(_write_config(tmp_path, toml))
        assert apply_view(cfg, None) is cfg
        assert {cage.view for cage in apply_view(cfg, "distorted").cages} == {"distorted"}
        with pytest.raises(ConfigError, match="--view"):
            apply_view(cfg, "fisheye")

    def test_shipped_cages_write_pinhole(self) -> None:
        assert {cage.view for cage in load_render_config(REAL_CONFIG).cages} == {"pinhole"}
