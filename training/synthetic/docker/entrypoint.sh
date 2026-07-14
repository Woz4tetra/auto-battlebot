#!/usr/bin/env bash
set -euo pipefail

# Step down to the host's UID/GID so files written into the mounted repo are owned
# by the host user instead of root. run_synthetic.sh passes HOST_UID/HOST_GID; when
# they are absent (image run directly) we keep the previous root behavior.
if [ "$(id -u)" = "0" ] && [ -n "${HOST_UID:-}" ] && [ -n "${HOST_GID:-}" ] && [ "${HOST_UID}" != "0" ]; then
  getent group "$HOST_GID" >/dev/null 2>&1 || groupadd -g "$HOST_GID" bot
  getent passwd "$HOST_UID" >/dev/null 2>&1 || \
    useradd -u "$HOST_UID" -g "$HOST_GID" -M -d /home/bot -s /bin/bash bot

  mkdir -p /home/bot
  chown "$HOST_UID:$HOST_GID" /home/bot

  # BlenderProc resolves Blender at $HOME/blender; the image bakes it under /root
  # (0700). Make /root traversable and link the baked install into the new home so
  # the dropped user reuses it instead of re-downloading.
  chmod o+rx /root
  [ -e /home/bot/blender ] || ln -s /root/blender /home/bot/blender

  # HF_HOME is a baked, root-owned dir unless a host cache is mounted over it; give
  # the target user ownership of the top level (a no-op for a host-owned mount) so
  # the cache is writable after the privilege drop.
  hf_home="${HF_HOME:-/opt/hf}"
  mkdir -p "$hf_home"
  chown "$HOST_UID:$HOST_GID" "$hf_home"

  export HOME=/home/bot USER=bot
  # Re-exec this script as the target user; the root branch above is then skipped.
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

# Prettier shell prompt for interactive terminal sessions.
if [ -t 1 ]; then
  export CLICOLOR=1
  export LS_COLORS="${LS_COLORS:-di=1;34:ln=1;36:so=1;35:pi=33:ex=1;32:bd=1;33:cd=1;33:su=37;41:sg=30;43:tw=30;42:ow=30;43}"
  if [ -z "${PS1:-}" ]; then
    export PS1='\[\e[1;38;5;39m\]\u@\h\[\e[0m\]:\[\e[1;38;5;214m\]\w\[\e[0m\]\$ '
  fi
fi

exec "$@"
