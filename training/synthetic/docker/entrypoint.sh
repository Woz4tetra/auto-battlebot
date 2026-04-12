#!/usr/bin/env bash
set -euo pipefail

default_cwd="/workspace/training/synthetic"
if [ -d "$default_cwd" ]; then
  cd "$default_cwd"
elif [ -d /workspace ]; then
  cd /workspace
fi

export PYTHONUNBUFFERED=1
export HF_HOME="${HF_HOME:-/opt/hf}"
export TRANSFORMERS_CACHE="${TRANSFORMERS_CACHE:-$HF_HOME/transformers}"

mkdir -p "$HF_HOME" "$TRANSFORMERS_CACHE"

if [ ! -f "$HF_HOME/.has_hf_login" ]; then
  echo "[synthetic-entrypoint] Optional first-time step: run 'huggingface-cli login' for SF3D."
  echo "[synthetic-entrypoint] If you already logged in, create $HF_HOME/.has_hf_login to silence this hint."
fi

# Prettier shell prompt for interactive terminal sessions.
if [ -t 1 ]; then
  export CLICOLOR=1
  export LS_COLORS="${LS_COLORS:-di=1;34:ln=1;36:so=1;35:pi=33:ex=1;32:bd=1;33:cd=1;33:su=37;41:sg=30;43:tw=30;42:ow=30;43}"
  if [ -z "${PS1:-}" ]; then
    export PS1='\[\e[1;38;5;39m\]\u@\h\[\e[0m\]:\[\e[1;38;5;214m\]\w\[\e[0m\]\$ '
  fi
fi

exec "$@"
