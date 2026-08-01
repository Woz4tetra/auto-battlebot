#!/bin/bash
# setup_claude_code.sh: wire up this repo's Claude Code integration -- the
# clangd-lsp plugin and the no-ai-slop wording hook.
#
# For normal interactive use you do NOT need this script for the plugin:
# .claude/settings.json already declares the `claude-plugins-official`
# marketplace and enables `clangd-lsp`, so Claude Code offers to install it on
# first launch after you trust the workspace. Run this only for headless /
# non-interactive setups (CI, remote boxes) where that prompt never fires.
#
# The wording hook needs no installation either -- .claude/settings.json points
# PostToolUse at scripts/no_ai_slop_check.sh. This script only checks that its
# dependencies are in place, since a hook that cannot run fails silently.
#
# The clangd binary itself is installed separately by the OS setup
# (install/base_packages.txt lists `clangd`); this script only wires the plugin.

set -e

MARKETPLACE="anthropics/claude-plugins-official"
PLUGIN="clangd-lsp@claude-plugins-official"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SLOP_HOOK="${REPO_ROOT}/scripts/no_ai_slop_check.sh"

if ! command -v claude >/dev/null 2>&1; then
    echo "Error: 'claude' CLI not found on PATH. Install Claude Code first." >&2
    exit 1
fi

if ! command -v clangd >/dev/null 2>&1; then
    echo "Warning: 'clangd' not found on PATH. The plugin will install but stay" >&2
    echo "         inactive until clangd is available (see install/base_packages.txt)." >&2
fi

echo "Registering marketplace: ${MARKETPLACE}"
claude plugin marketplace add "${MARKETPLACE}" 2>/dev/null || true

echo "Installing plugin (project scope): ${PLUGIN}"
claude plugin install "${PLUGIN}" --scope project

echo "Checking the no-ai-slop wording hook"
if [ ! -f "${SLOP_HOOK}" ]; then
    echo "Error: ${SLOP_HOOK} is missing. The PostToolUse hook declared in" >&2
    echo "       .claude/settings.json will fail on every file write." >&2
    exit 1
fi

# git preserves the exec bit, but a zip export or a restrictive umask does not.
[ -x "${SLOP_HOOK}" ] || chmod +x "${SLOP_HOOK}"

if ! command -v jq >/dev/null 2>&1; then
    echo "Warning: 'jq' not found on PATH. The wording hook parses its input with" >&2
    echo "         jq and will pass everything silently until it is installed" >&2
    echo "         (see install/base_packages.txt)." >&2
fi

echo "Done. clangd-lsp and the no-ai-slop wording hook are enabled for this repository."
