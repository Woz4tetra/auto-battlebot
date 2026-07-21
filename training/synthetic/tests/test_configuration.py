"""Tests for the typed config layer against the real config.toml and edge cases."""

from pathlib import Path

import numpy as np
import pytest
from synthgen.configuration import (
    ConfigError,
    PathResolver,
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
        assert stabs.name == "MR_STABS_MK2"
        assert stabs.class_id == 0
        assert stabs.weight == pytest.approx(0.5)
        assert stabs.ground_roll_upright == pytest.approx(9.587585)
        # Keypoints are converted to Blender axes: (x, y, z) -> (x, -z, y).
        np.testing.assert_allclose(stabs.keypoints.front, [0.1115, 0.00313, 0.0])
        assert buff.name == "MRS_BUFF_MK3"
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
        assert cfg.distractors.robot_air_probability == pytest.approx(0.3)
        assert cfg.distractors.has_cad_source() is True
        kinds = [s.effective_kind() for s in cfg.distractors.sources]
        assert "cad" in kinds

        assert cfg.camera.max_distance == pytest.approx(6.0)
        assert cfg.camera.max_frame_fraction == pytest.approx(0.85)
        assert cfg.scene.max_robots_per_scene == 10
        assert cfg.randomization.air_probability == pytest.approx(0.3)
        assert cfg.randomization.motion_blur_strength_range == (5, 15)

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
