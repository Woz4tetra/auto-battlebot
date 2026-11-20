#!/usr/bin/env python3
"""Entry point for the serial GPU job queue in auto_battlebot/gpu_queue.py."""

import sys

from auto_battlebot.gpu_queue import main

if __name__ == "__main__":
    sys.exit(main())
