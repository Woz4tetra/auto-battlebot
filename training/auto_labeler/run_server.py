#!/usr/bin/env python3
"""Run the video labeling server."""

import argparse
import sys

from labeling_server import run_server


def main():
    parser = argparse.ArgumentParser(
        description="Run the video labeling server",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example:
    python run_server.py --config config/labeling_config.yaml
    
The server will load the video and SAM3 model, then listen for client connections.
""",
    )
    parser.add_argument(
        "--config", "-c",
        default="config/labeling_config.yaml",
        help="Path to config file (default: config/labeling_config.yaml)",
    )
    
    args = parser.parse_args()
    
    try:
        from labeling_server.server import run_server
        run_server(args.config)
    except FileNotFoundError as e:
        print(f"Error: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nServer stopped")
        sys.exit(0)


if __name__ == "__main__":
    main()
