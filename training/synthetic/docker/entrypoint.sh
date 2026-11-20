#!/usr/bin/env bash
set -euo pipefail

link_baked_blender_into_home() {
  chmod o+rx /root
  [ -e /home/bot/blender ] || ln -s /root/blender /home/bot/blender
}

hand_hf_cache_to_host_user() {
  mkdir -p "$1"
  chown "$HOST_UID:$HOST_GID" "$1"
}

if [ "$(id -u)" = "0" ] && [ -n "${HOST_UID:-}" ] && [ -n "${HOST_GID:-}" ] && [ "${HOST_UID}" != "0" ]; then
  getent group "$HOST_GID" >/dev/null 2>&1 || groupadd -g "$HOST_GID" bot
  getent passwd "$HOST_UID" >/dev/null 2>&1 || \
    useradd -u "$HOST_UID" -g "$HOST_GID" -M -d /home/bot -s /bin/bash bot

  mkdir -p /home/bot
  chown "$HOST_UID:$HOST_GID" /home/bot

  link_baked_blender_into_home
  hf_home="${HF_HOME:-/opt/hf}"
  hand_hf_cache_to_host_user "$hf_home"

  export HOME=/home/bot USER=bot
  exec setpriv --reuid "$HOST_UID" --regid "$HOST_GID" --clear-groups "$0" "$@"
fi

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
  echo "[synthetic-entrypoint] Optional first-time step: run 'huggingface-cli login' for asset downloads."
  echo "[synthetic-entrypoint] If you already logged in, create $HF_HOME/.has_hf_login to silence this hint."
fi

if [ -t 1 ]; then
  export CLICOLOR=1
  export LS_COLORS="${LS_COLORS:-di=1;34:ln=1;36:so=1;35:pi=33:ex=1;32:bd=1;33:cd=1;33:su=37;41:sg=30;43:tw=30;42:ow=30;43}"
  if [ -z "${PS1:-}" ]; then
    export PS1='\[\e[1;38;5;39m\]\u@\h\[\e[0m\]:\[\e[1;38;5;214m\]\w\[\e[0m\]\$ '
  fi
fi

exec "$@"
