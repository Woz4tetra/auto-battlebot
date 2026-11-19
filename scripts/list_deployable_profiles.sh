#!/bin/bash
set -e
config_dir="$1"
pattern="$2"
cd "$config_dir"
find . -type f -name '*.toml' \
    | sed -e 's|^\./||' -e 's|\.toml$||' | grep -xE "$pattern" | LC_ALL=C sort -u
