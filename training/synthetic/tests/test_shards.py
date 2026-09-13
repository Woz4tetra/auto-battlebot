"""Tests for the sharded render plan and merge (synthgen.shards)."""

import json
import os

import pytest
from synthgen.shards import (
    ShardRun,
    existing_frames,
    merge_parts,
    plan_shard_runs,
    resume_point,
    split_evenly,
)

NHRL_ORDER = ("pinhole", "rectified", "distorted")
MASSD_ORDER = ("rectified", "distorted", "pinhole")


def _counts(runs, venue_views):
    """{shard: [count per view in venue_views order]}."""
    table = {}
    for run in runs:
        table.setdefault(run.shard, {})[run.view] = run.num_images
    return {shard: [row[view] for view in venue_views] for shard, row in table.items()}


class TestSplitEvenly:
    def test_even_split(self):
        assert split_evenly(9, 3) == [3, 3, 3]

    def test_remainder_goes_first(self):
        assert split_evenly(20000, 3) == [6667, 6667, 6666]

    def test_offset_moves_the_remainder(self):
        assert split_evenly(6667, 3, offset=1) == [2222, 2223, 2222]
        assert split_evenly(5, 3, offset=2) == [2, 1, 2]

    def test_rejects_bad_input(self):
        with pytest.raises(ValueError):
            split_evenly(5, 0)
        with pytest.raises(ValueError):
            split_evenly(-1, 3)


class TestPlanShardRuns:
    def test_nhrl_matches_the_plan_table(self):
        runs = plan_shard_runs(20000, 3, NHRL_ORDER, seed_base=0)
        assert _counts(runs, ("pinhole", "rectified", "distorted")) == {
            0: [2223, 2222, 2222],
            1: [2222, 2223, 2222],
            2: [2222, 2222, 2222],
        }

    def test_massd_matches_the_plan_table(self):
        runs = plan_shard_runs(20000, 3, MASSD_ORDER, seed_base=200)
        assert _counts(runs, ("pinhole", "rectified", "distorted")) == {
            0: [2222, 2223, 2222],
            1: [2222, 2222, 2223],
            2: [2222, 2222, 2222],
        }

    def test_both_venues_reach_13333_per_view(self):
        totals = {}
        for order in (NHRL_ORDER, MASSD_ORDER):
            for run in plan_shard_runs(20000, 3, order, seed_base=0):
                totals[run.view] = totals.get(run.view, 0) + run.num_images
        assert totals == {"pinhole": 13333, "rectified": 13334, "distorted": 13333}

    def test_shards_stay_within_one_frame(self):
        runs = plan_shard_runs(20000, 3, NHRL_ORDER, seed_base=0)
        per_shard = [sum(r.num_images for r in runs if r.shard == s) for s in range(3)]
        assert per_shard == [6667, 6667, 6666]

    def test_frame_ranges_are_contiguous_and_disjoint(self):
        runs = plan_shard_runs(20000, 3, NHRL_ORDER, seed_base=0)
        assert runs[0].start_index == 0
        for before, after in zip(runs, runs[1:]):
            assert after.start_index == before.end_index
        assert runs[-1].end_index == 20000

    def test_seeds_are_distinct_and_offset(self):
        runs = plan_shard_runs(18, 3, NHRL_ORDER, seed_base=200)
        seeds = [run.seed for run in runs]
        assert seeds == list(range(200, 209))

    def test_rejects_unknown_or_repeated_views(self):
        with pytest.raises(ValueError, match="unknown"):
            plan_shard_runs(10, 2, ("pinhole", "fisheye"), 0)
        with pytest.raises(ValueError, match="distinct"):
            plan_shard_runs(10, 2, ("pinhole", "pinhole"), 0)
        with pytest.raises(ValueError, match="distinct"):
            plan_shard_runs(10, 2, (), 0)

    def test_run_names_carry_shard_and_view(self):
        assert ShardRun(2, "distorted", 0, 1, 0).name == "shard2_distorted"


class TestResume:
    def test_existing_frames_keeps_only_the_runs_range(self, tmp_path):
        images = tmp_path / "images"
        images.mkdir()
        for index in (4, 5, 6, 9, 10):
            (images / f"{index:06d}.jpg").write_bytes(b"x")
        (images / "notes.jpg").write_bytes(b"x")
        run = ShardRun(0, "pinhole", start_index=5, num_images=5, seed=0)
        assert existing_frames(images, run) == [5, 6, 9]

    def test_missing_directory_has_no_frames(self, tmp_path):
        assert existing_frames(tmp_path / "nope", ShardRun(0, "pinhole", 0, 3, 0)) == []

    def test_resume_point(self):
        run = ShardRun(1, "rectified", start_index=100, num_images=50, seed=0)
        assert resume_point([], run) == (100, 50)
        assert resume_point([100, 101, 120], run) == (121, 29)
        assert resume_point(list(range(100, 150)), run) == (150, 0)


def _make_part(root, name, indices, view, venue="nhrl_cage", data_path=None):
    part = root / name
    (part / "images").mkdir(parents=True)
    (part / "labels").mkdir()
    rows = []
    for index in indices:
        stem = f"{index:06d}"
        (part / "images" / f"{stem}.jpg").write_bytes(b"jpg")
        (part / "labels" / f"{stem}.txt").write_text("0 0.5 0.5 0.1 0.1\n")
        rows.append(json.dumps({"image": f"{stem}.jpg", "venue": venue, "view": view}))
    (part / "manifest.jsonl").write_text("\n".join(rows) + "\n")
    path = data_path or f"/workspace/training/data/{name}"
    (part / "data.yml").write_text(f"path: {path}\ntrain: images\nval: images\nnc: 4\n")
    return part


class TestMergeParts:
    def test_links_frames_and_concatenates_manifests(self, tmp_path):
        a = _make_part(tmp_path, "shard0_pinhole", [0, 1], "pinhole")
        b = _make_part(tmp_path, "shard0_distorted", [2, 3, 4], "distorted")
        out = tmp_path / "merged"
        summary = merge_parts([a, b], out)

        assert summary.images == 5
        assert summary.per_view == {"distorted": 3, "pinhole": 2}
        assert summary.per_venue == {"nhrl_cage": 5}
        merged = out / "images" / "000003.jpg"
        assert os.stat(merged).st_ino == os.stat(b / "images" / "000003.jpg").st_ino
        assert (out / "labels" / "000000.txt").is_file()
        lines = (out / "manifest.jsonl").read_text().splitlines()
        assert [json.loads(line)["image"] for line in lines] == [
            "000000.jpg",
            "000001.jpg",
            "000002.jpg",
            "000003.jpg",
            "000004.jpg",
        ]
        data_yml = (out / "data.yml").read_text().splitlines()
        assert data_yml[0] == f"path: {out.resolve()}"
        assert data_yml[1:] == ["train: images", "val: images", "nc: 4"]

    def test_debug_frames_are_left_behind(self, tmp_path):
        part = _make_part(tmp_path, "a", [0, 1], "pinhole")
        (part / "images" / "_debug_frame0.jpg").write_bytes(b"jpg")
        out = tmp_path / "merged"
        assert merge_parts([part], out).images == 2
        assert not (out / "images" / "_debug_frame0.jpg").exists()

    def test_rows_without_a_view_are_counted_as_unrecorded(self, tmp_path):
        part = _make_part(tmp_path, "old", [0], "pinhole")
        row = json.loads((part / "manifest.jsonl").read_text())
        del row["view"]
        (part / "manifest.jsonl").write_text(json.dumps(row) + "\n")
        assert merge_parts([part], tmp_path / "merged").per_view == {"unrecorded": 1}

    def test_repeated_file_name_writes_nothing(self, tmp_path):
        a = _make_part(tmp_path, "a", [0, 1], "pinhole")
        b = _make_part(tmp_path, "b", [1, 2], "distorted")
        out = tmp_path / "merged"
        with pytest.raises(ValueError, match="000001.jpg"):
            merge_parts([a, b], out)
        assert not out.exists()

    def test_missing_label_fails(self, tmp_path):
        part = _make_part(tmp_path, "a", [0, 1], "pinhole")
        (part / "labels" / "000001.txt").unlink()
        with pytest.raises(ValueError, match="no label"):
            merge_parts([part], tmp_path / "merged")

    def test_manifest_out_of_step_with_images_fails(self, tmp_path):
        part = _make_part(tmp_path, "a", [0, 1], "pinhole")
        (part / "images" / "000001.jpg").unlink()
        (part / "labels" / "000001.txt").unlink()
        with pytest.raises(ValueError, match="manifest"):
            merge_parts([part], tmp_path / "merged")

    def test_repeated_manifest_row_fails(self, tmp_path):
        part = _make_part(tmp_path, "a", [0], "pinhole")
        line = (part / "manifest.jsonl").read_text()
        (part / "manifest.jsonl").write_text(line + line)
        with pytest.raises(ValueError, match="repeated"):
            merge_parts([part], tmp_path / "merged")

    def test_data_yml_mismatch_beyond_path_fails(self, tmp_path):
        a = _make_part(tmp_path, "a", [0], "pinhole")
        b = _make_part(tmp_path, "b", [1], "pinhole")
        (b / "data.yml").write_text("path: /elsewhere\ntrain: images\nval: images\nnc: 3\n")
        with pytest.raises(ValueError, match="data.yml"):
            merge_parts([a, b], tmp_path / "merged")

    def test_refuses_an_existing_merge(self, tmp_path):
        part = _make_part(tmp_path, "a", [0], "pinhole")
        out = tmp_path / "merged"
        (out / "images").mkdir(parents=True)
        with pytest.raises(ValueError, match="already exists"):
            merge_parts([part], out)
