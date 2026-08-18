"""Shared library modules for the calibration tools.

Imported, not run directly; the executable scripts stay one level up.

Velocity jig path (`velocity_jig_drive.py`, `fit_jig_plant.py`):
  jig_link        the jig's USB console: LIST/GET/DEL/TIME/STREAM, clock probes
  excitation      waveform generators and the TOML catalog
  drive_protocol  the OpenTX trainer link, writing commands and reading them back

AprilTag path (offline analysis of existing recordings):
  apriltag_detect AprilTag detection and field-plane pose geometry
  apriltag_mcap   the MCAP recording layout
"""
