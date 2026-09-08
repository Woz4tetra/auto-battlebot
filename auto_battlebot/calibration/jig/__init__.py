"""Velocity jig: drive the robot through waveforms and fit a plant to what came back.

velocity_jig    log parsing, IMU/encoder calibration, dead-reckoned ground truth
jig_link        the jig's USB console: LIST/GET/DEL/TIME/STREAM, clock probes
excitation      waveform generators and the TOML catalog
drive_protocol  the OpenTX trainer link, writing commands and reading them back
jig_fit         per-phase and joint plant fits over jig sessions
"""
