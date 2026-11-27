#!/bin/bash
set -euo pipefail

data="$HOME/ci-data"

ln -sfn "$HOME/venv" venv
rm -rf data/svo data/models third_party/foxglove
mkdir -p data third_party
ln -s "$data/svo" data/svo
ln -s "$data/models" data/models
ln -s "$data/third_party/foxglove" third_party/foxglove

venv/bin/pip install -q -e . --no-deps
venv/bin/python -c "import auto_battlebot, sys; sys.exit(not auto_battlebot.__file__.startswith('$PWD'))"

npm ci --prefix web --cache "$data/npm" --no-audit --no-fund
