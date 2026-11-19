# BrettZone Cage-N-Overhead-High recordings

`scripts/nhrl_to_recordings.py` turns NHRL fixed-cage fight video into a replayable recording set. It fetches the fights (optional), re-wraps each clip as a recording the RGB camera could have written, lands them in `data/saved_recordings/<set>/`, and writes the playback config that points at them. Re-running skips clips already converted, so adding fights to a set costs only the new ones.

NHRL's `Cage-N-Overhead-High` is the one angle that does not move: one fixed 1080p view, no depth, no visual odometry. It is the closest thing to the RGB camera that exists before the camera does. Every other recording in the repo is a moving ZED.

```bash
venv/bin/python scripts/nhrl_to_recordings.py --from-dir data/downloads/mrsbuff_may26
venv/bin/python scripts/nhrl_to_recordings.py --bot "MRS BUFF" --since 2026-06-01 --limit 6
venv/bin/python scripts/nhrl_to_recordings.py --from-dir dir/ --name BrettZone_2026-06-14
```

## Caveats

- The intrinsics in `config/cameras/brettzone_cage_high.toml` are backed out of NHRL's stated field of view, not measured. The homography field fit inherits that error linearly, so ranges off these recordings are indicative, not trusted.
- The detectors were trained on ZED frames at a different mount, so detections here report transfer, not accuracy.

## Fitted value: `DEFAULT_FOCAL_PX = 1331.0`

NHRL shoots this angle with an iPhone Pro at 1x, about 71.6 degrees horizontal. At 1920 px wide that backs out to `960 / tan(35.8 deg)`. It is an estimate, not a calibration.

## Set names

BrettZone exports are named `BZ-<tournament>-<p1>-<p2>-<game>-Cage-<n>-Overhead-High.mp4`, and the tournament id carries the event month: `nhrl_may26` is NHRL May 2026, not the 26th. Sibling sets in `data/saved_recordings` are named `<Venue>_<date>`, so the script produces `BrettZone_2026-05` rather than NHRL's own shorthand.

## Provenance manifest

`data/` is gitignored, so the manifest written next to the recordings is the only place that says where they came from and what was assumed about the camera. Without it a recording set is anonymous in six months. Frame counts are read from an existing recording's summary instead of assumed, so the manifest says the same thing whether a clip was converted now or last week.

## Training corpus overlap

The 2class models were trained on `nhrl_robots_bbox_2class`, whose train split holds most of the labelled cage-high scenes. The downloader lists the tarball with `tar tf` to learn which tournaments those scenes came from, because extracting it to look would take 12 GB. A missing tarball and a tarball with no BrettZone-style `tournamentID-` names are different answers, and `MANIFEST.md` reports each one separately.

## Playback config

The generated config points at the first recording. `MANIFEST.md` next to the recordings lists the rest; switching clips is pasting one of those paths into `video_file_path`.
