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
# The no-ai-slop skill is a user-level install (~/.claude/skills/), not part of
# this repo, so a fresh machine does not have it. This script clones it when it
# is missing. The repo still works without it: CLAUDE.md carries the writing
# rules and the hook carries the wordlist. The skill adds the /no-ai-slop
# command for editing a draft on demand.
#
# The clangd binary itself is installed separately by the OS setup
# (install/base_packages.txt lists `clangd`); this script only wires the plugin.

set -e

MARKETPLACE="anthropics/claude-plugins-official"
PLUGIN="clangd-lsp@claude-plugins-official"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SLOP_HOOK="${REPO_ROOT}/scripts/no_ai_slop_check.sh"

SKILL_REPO="https://github.com/petergyang/no-ai-slop"
# The three files the skill needs; the upstream repo keeps them at its root.
SKILL_FILES=(SKILL.md eval.md LICENSE)
SKILL_DIR="${CLAUDE_CONFIG_DIR:-${HOME}/.claude}/skills/no-ai-slop"

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

echo "Checking the no-ai-slop skill"
if [ -f "${SKILL_DIR}/SKILL.md" ]; then
    echo "  present at ${SKILL_DIR}"
elif ! command -v git >/dev/null 2>&1; then
    echo "Warning: 'git' not found on PATH, so the no-ai-slop skill cannot be" >&2
    echo "         installed. Copy ${SKILL_FILES[*]} from ${SKILL_REPO}" >&2
    echo "         into ${SKILL_DIR}/ by hand." >&2
else
    echo "  missing, cloning ${SKILL_REPO}"
    SKILL_TMP="$(mktemp -d)"
    trap 'rm -rf "${SKILL_TMP}"' EXIT

    skill_ok=1
    # GIT_TERMINAL_PROMPT=0 so a bad URL fails instead of blocking a headless
    # setup on a credential prompt.
    GIT_TERMINAL_PROMPT=0 git clone --depth 1 --quiet "${SKILL_REPO}" "${SKILL_TMP}" || skill_ok=0
    for f in "${SKILL_FILES[@]}"; do
        [ "${skill_ok}" = 1 ] && [ -f "${SKILL_TMP}/${f}" ] || skill_ok=0
    done

    if [ "${skill_ok}" = 1 ]; then
        mkdir -p "${SKILL_DIR}"
        for f in "${SKILL_FILES[@]}"; do
            cp "${SKILL_TMP}/${f}" "${SKILL_DIR}/"
        done
        echo "  installed ${SKILL_FILES[*]} to ${SKILL_DIR}"
    else
        # Not fatal: CLAUDE.md and the hook enforce the rules without the skill.
        echo "Warning: could not fetch the no-ai-slop skill from ${SKILL_REPO}." >&2
        echo "         The writing rules in CLAUDE.md and the wording hook still" >&2
        echo "         apply; only the /no-ai-slop command is unavailable." >&2
    fi
fi

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

echo "Done. clangd-lsp, the no-ai-slop skill and the wording hook are set up for"
echo "this repository."
