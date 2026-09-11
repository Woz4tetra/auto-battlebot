#!/usr/bin/env bash
# Stage 3 of the cage HDRI pipeline: example BlenderProc renders lit only by the
# stitched HDRI.
#
# Derives a config from training/synthetic/config.toml with the HDRI directory
# pointed at the stitch output, all point lights disabled so the HDRI is the only
# light, and a small image count. Runs it through the synthetic Docker image with
# GPU passthrough. Asset paths in the base config are relative to
# training/synthetic, which is the container working directory, so they resolve
# unchanged.
#
# Usage:
#   playground/cage_hdri/render_examples.sh [HDRI_DIR] [OUT_DIR] [NUM_IMAGES]
#
# Defaults: runs/cage_hdri/hdri runs/cage_hdri/render 12

set -euo pipefail

hdri_dir="${1:-runs/cage_hdri/hdri}"
out_dir="${2:-runs/cage_hdri/render}"
num_images="${3:-12}"
# Anything after the three positionals goes to render_scenes.py (e.g. --render-samples).
shift $(( $# < 3 ? $# : 3 ))
image_name="auto-battlebot-synthetic"

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_root"

hdris=("$hdri_dir"/*.exr "$hdri_dir"/*.hdr)
hdris=("${hdris[@]/#*\**/}")  # drop unmatched globs, which bash leaves literal
if [ -z "${hdris[*]}" ]; then
    echo "no .exr or .hdr in $hdri_dir, run stitch_hdri.py first" >&2
    exit 1
fi

# The config resolver tries the config's own directory, then the launch cwd, then
# the repo root, and returns the first that exists. Output dirs are created here so
# the config-relative candidate wins for them.
mkdir -p "$out_dir/images" "$out_dir/labels"
config="$out_dir/config.toml"

sed \
    -e "s|^hdri_dir = .*|hdri_dir = \"$hdri_dir\"|" \
    -e 's|^image_dir = .*|image_dir = "images"|' \
    -e 's|^label_dir = .*|label_dir = "labels"|' \
    -e "s|^num_images = .*|num_images = $num_images|" \
    -e 's|^images_per_scene = .*|images_per_scene = 3|' \
    -e 's|^light_count_range = .*|light_count_range = [0, 0]|' \
    -e 's|^ground_visibility = .*|ground_visibility = 0.5|' \
    training/synthetic/config.toml > "$config"

echo "config: $config"
echo "hdri:   ${hdris[*]}"

# Blender's bundled Python ignores PYTHONPATH, so the wrapper's PYTHONPATH never
# reaches render_scenes.py and `from synthgen import ...` fails. This shim puts
# training/synthetic on sys.path first, then runs the real script in place.
shim="$out_dir/run_render_scenes.py"
cat > "$shim" <<'EOF'
import blenderproc as bproc  # noqa: F401  (blenderproc requires this first import)
import runpy
import sys

sys.path.insert(0, "/workspace/training/synthetic")
runpy.run_path("/workspace/training/synthetic/render_scenes.py", run_name="__main__")
EOF

training/synthetic/docker/run_synthetic.sh --gpu "$image_name" \
    blenderproc run "/workspace/$shim" -- "/workspace/$config" --seed 0 "$@"

echo "renders: $out_dir/images"
ls "$out_dir/images" | head
