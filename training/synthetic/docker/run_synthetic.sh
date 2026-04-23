#!/usr/bin/env bash
set -euo pipefail

if [ $# -lt 1 ]; then
  cat <<'EOF'
Usage:
  training/synthetic/docker/run_synthetic.sh [--gpu] [--require-gpu] [--cpu] <image_name> [args...]

Examples:
  training/synthetic/docker/run_synthetic.sh auto-battlebot-synthetic
  training/synthetic/docker/run_synthetic.sh --gpu auto-battlebot-synthetic
  training/synthetic/docker/run_synthetic.sh --cpu auto-battlebot-synthetic
  training/synthetic/docker/run_synthetic.sh --require-gpu auto-battlebot-synthetic
  training/synthetic/docker/run_synthetic.sh auto-battlebot-synthetic python training/synthetic/download_objaverse.py --help
  training/synthetic/docker/run_synthetic.sh auto-battlebot-synthetic blenderproc run training/synthetic/render_scenes.py -- training/synthetic/config.toml --num-images 4
EOF
  exit 1
fi

docker_gpu_args=()
require_gpu=0
while [ $# -gt 0 ]; do
  case "${1:-}" in
    --gpu)
      docker_gpu_args=(--gpus all)
      shift
      ;;
    --cpu)
      docker_gpu_args=()
      shift
      ;;
    --require-gpu)
      require_gpu=1
      shift
      ;;
    *)
      break
      ;;
  esac
done

if [ $# -lt 1 ]; then
  echo "Error: image name is required."
  exit 1
fi

image_name="$1"
shift

if [ $# -eq 0 ]; then
  set -- /bin/bash
fi

hf_cache_host="${HOME}/.cache/huggingface"
mkdir -p "$hf_cache_host"

# Resolve repository root from this script location so behavior is stable
# regardless of the caller's current working directory.
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/../../.." && pwd)"

docker_tty_args=()
if [ -t 0 ] && [ -t 1 ]; then
  docker_tty_args=(-it)
fi

run_cmd=(
  docker run "${docker_gpu_args[@]}" --rm "${docker_tty_args[@]}"
  -v "${repo_root}:/workspace"
  -v "${hf_cache_host}:/opt/hf"
  -w /workspace/training/synthetic
  "$image_name" "$@"
)

if [ "${#docker_gpu_args[@]}" -eq 0 ]; then
  "${run_cmd[@]}"
  exit $?
fi

# Probe GPU runtime support first to preserve interactive behavior and signals.
set +e
gpu_probe_output="$(docker run --rm --gpus all --entrypoint /bin/sh "$image_name" -c "exit 0" 2>&1)"
gpu_probe_exit=$?
set -e

if [ $gpu_probe_exit -ne 0 ]; then
  if [ "$require_gpu" -eq 1 ]; then
    echo "GPU is required but Docker GPU runtime is unavailable." >&2
    [ -n "${gpu_probe_output:-}" ] && echo "$gpu_probe_output" >&2
    echo "Tip: install/configure NVIDIA Container Toolkit." >&2
    exit $gpu_probe_exit
  fi

  echo "Docker GPU runtime unavailable; falling back to CPU mode." >&2
  [ -n "${gpu_probe_output:-}" ] && echo "$gpu_probe_output" >&2
  echo "Tip: install/configure NVIDIA Container Toolkit for GPU runs." >&2
  docker run --rm "${docker_tty_args[@]}" \
    -v "${repo_root}:/workspace" \
    -v "${hf_cache_host}:/opt/hf" \
    -w /workspace/training/synthetic \
    "$image_name" "$@"
  exit $?
fi

"${run_cmd[@]}"
