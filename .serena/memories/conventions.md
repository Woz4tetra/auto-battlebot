# Code conventions

## C++
- Google style, 4-space indent, 100-char line limit (.clang-format).
- All code in namespace `auto_battlebot`.
- Interface in `include/<module>/`, impl in `src/<module>/`, wired via the module factory.
- Prefer TOML config over compile-time switches / defensive config flags. This project has
  a single consumer: pick the correct behavior and hard-code it rather than adding knobs.
- -Wall -Wextra -Werror: fix warnings, do not suppress.
- No blocking calls in the perception loop.

## Python
- ruff for lint/format; mypy for types (strict enough that new code must pass).
- Analysis/training tooling in `training/`; shared logic in the `auto_battlebot/` package.
- YOLO annotation format.

## Working tree
- User co-edits this tree. Be surgical: only touch/format files you intentionally changed.
- Persist app-written runtime state under $HOME, not config/ (deploys clobber the repo).