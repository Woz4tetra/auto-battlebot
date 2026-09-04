"""Calibration library: jig drive, AprilTag ground truth, and plant window loaders.

Imported, not run directly. The command-line tools over this live in `playground/calibration/`.

Velocity jig path (`velocity_jig_drive.py`, `fit_jig_plant.py`):
  jig_link        the jig's USB console: LIST/GET/DEL/TIME/STREAM, clock probes
  excitation      waveform generators and the TOML catalog
  drive_protocol  the OpenTX trainer link, writing commands and reading them back
  jig_fit         per-phase and joint plant fits over jig sessions

AprilTag path (offline analysis of existing recordings):
  apriltag_detect AprilTag detection and field-plane pose geometry
  apriltag_mcap   the MCAP recording layout

Match path (plant fit from recorded fights):
  match_windows   joins replay poses to transmitted commands, cuts scoring windows
"""
