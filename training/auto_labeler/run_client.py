#!/usr/bin/env python3
"""Run the video labeling client."""

import argparse
import sys


def main():
    parser = argparse.ArgumentParser(
        description="Run the video labeling client",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example:
    python run_client.py --server http://192.168.1.100:8765
    
The client will connect to the server and open the labeling interface.

Keyboard shortcuts:
    Left/Right      - Previous/next frame
    Shift+Left/Right - Jump 10 frames
    Home/End        - First/last frame
    Ctrl+Z          - Undo last point
    Space           - Preview mask
    Enter           - Propagate masks
    Escape          - Toggle add/remove mode
    1-9             - Select object label
""",
    )
    parser.add_argument(
        "--server",
        "-s",
        default="http://localhost:8765",
        help="Server URL (default: http://localhost:8765)",
    )

    args = parser.parse_args()

    try:
        from labeling_client.ui import run_client

        run_client(args.server)
    except KeyboardInterrupt:
        print("\nClient stopped")
        sys.exit(0)


if __name__ == "__main__":
    main()
