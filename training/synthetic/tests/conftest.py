"""Pytest setup for the synthgen tests.

synthgen and nhrl_common are importable because scripts/setup_python.sh adds
training/synthetic to the venv's import path (auto_battlebot.pth in
site-packages), so no sys.path manipulation is needed here.
"""
