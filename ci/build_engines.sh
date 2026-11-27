#!/bin/bash
set -euo pipefail

exec venv/bin/python -m auto_battlebot.verify.ci_engines build "$@"
