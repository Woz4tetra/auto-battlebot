# Claude Code setup

`scripts/setup_claude_code.sh` wires up this repo's Claude Code integration: the clangd-lsp plugin and the no-ai-slop wording hook.

- For normal interactive use you do not need the script for the plugin. `.claude/settings.json` already declares the `claude-plugins-official` marketplace and enables `clangd-lsp`, so Claude Code offers to install it on first launch after you trust the workspace. Run the script only for headless or non-interactive setups (CI, remote boxes) where that prompt never fires.
- The wording hook needs no installation either. `.claude/settings.json` points PostToolUse at `scripts/no_ai_slop_check.sh`. The script only checks that the hook's dependencies are in place, since a hook that cannot run fails silently.
- The no-ai-slop skill is a user-level install (`~/.claude/skills/`), not part of this repo, so a fresh machine does not have it. The script clones it when it is missing. The repo still works without it: `CLAUDE.md` carries the writing rules and the hook carries the wordlist. The skill adds the `/no-ai-slop` command for editing a draft on demand.
- The clangd binary is installed separately by the OS setup (`install/base_packages.txt` lists `clangd`). The script only wires the plugin.

## no_ai_slop_check.sh

`scripts/no_ai_slop_check.sh` is the PostToolUse (Write and Edit) hook. It flags AI-slop wording in prose files and in the comment lines of source files. It reads the hook payload on stdin, scans the file that was just written, and on a hit prints the offending lines and exits 2, which feeds them back to Claude as an error to fix before it continues.

Run it by hand on files by passing them as arguments. With no arguments it reads the hook payload, and with no arguments and an empty or terminal stdin it exits 2 instead of passing with nothing checked.

```bash
scripts/no_ai_slop_check.sh docs/setup/claude_code_setup.md docs/README.md
```

Rules come from the no-ai-slop skill (`~/.claude/skills/no-ai-slop/SKILL.md`), whose wording section is also inlined in `CLAUDE.md`. Only the flat wordlist is enforced here. The structural patterns (colon reveals, binary contrasts, kicker endings) are not greppable and are left to `CLAUDE.md`.
