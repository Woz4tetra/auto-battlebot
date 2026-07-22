#!/bin/bash
# setup_claude_code.sh: install the clangd-lsp Claude Code plugin for this repo.
#
# For normal interactive use you do NOT need this script: .claude/settings.json
# already declares the `claude-plugins-official` marketplace and enables
# `clangd-lsp`, so Claude Code offers to install it on first launch after you
# trust the workspace. Run this only for headless / non-interactive setups
# (CI, remote boxes) where that prompt never fires.
#
# The clangd binary itself is installed separately by the OS setup
# (install/base_packages.txt lists `clangd`); this script only wires the plugin.

set -e

MARKETPLACE="anthropics/claude-plugins-official"
PLUGIN="clangd-lsp@claude-plugins-official"

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

echo "Done. clangd-lsp is enabled for this repository."
