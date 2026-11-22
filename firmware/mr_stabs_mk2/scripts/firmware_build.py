# noqa: INP001
"""PlatformIO pre-build step: define FIRMWARE_BUILD as the short git hash, -dirty if edited."""

import subprocess
from pathlib import Path

FIRMWARE_BUILD_LENGTH = 16


def firmware_build(repo: Path) -> str:
    """Short git hash of `repo`, suffixed -dirty when tracked files differ from HEAD."""
    try:
        commit = subprocess.run(
            ["git", "-C", str(repo), "rev-parse", "--short=8", "HEAD"],
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip()
        dirty = subprocess.run(
            ["git", "-C", str(repo), "status", "--porcelain", "--untracked-files=no"],
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"
    build = f"{commit}-dirty" if dirty else commit
    return build[:FIRMWARE_BUILD_LENGTH]


try:
    Import("env")  # noqa: F821
except NameError:
    env = None

if env is not None:
    build = firmware_build(Path(env.subst("$PROJECT_DIR")))
    env.Append(CPPDEFINES=[("FIRMWARE_BUILD", env.StringifyMacro(build))])
