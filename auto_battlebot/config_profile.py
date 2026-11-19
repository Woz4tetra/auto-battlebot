"""Load an app profile the way src/config/config.cpp does: ``extends`` chains and ``from`` files."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from auto_battlebot.compat import tomllib

CONFIG_ROOT = Path(__file__).resolve().parents[1] / "config"
FROM_KEY = "from"


def _merge(base: dict[str, Any], overlay: dict[str, Any]) -> None:
    for key, value in overlay.items():
        current = base.get(key)
        if isinstance(value, dict) and isinstance(current, dict):
            if "type" in value and "type" in current and value["type"] != current["type"]:
                base[key] = value
            else:
                _merge(current, value)
        else:
            base[key] = value


def load_profile_toml(name: str, config_root: Path = CONFIG_ROOT) -> dict[str, Any]:
    """The merged TOML of a profile, by name (``mr_stabs_mk2_sysid_zed_box``) or path."""
    path = Path(name)
    if path.suffix != ".toml":
        path = config_root / f"{name}.toml"
    chain: list[Path] = []
    return _load(path, config_root, chain)


def _resolve(config_root: Path, value: str) -> Path:
    path = config_root / value
    if path.suffix != ".toml":
        path = path.with_name(path.name + ".toml")
    return path


def _contains_from_key(table: dict[str, Any]) -> bool:
    return any(
        key == FROM_KEY or (isinstance(value, dict) and _contains_from_key(value))
        for key, value in table.items()
    )


def _find_table(root: dict[str, Any], section: tuple[str, ...]) -> dict[str, Any] | None:
    table: Any = root
    for part in section:
        table = table.get(part) if isinstance(table, dict) else None
    return table if isinstance(table, dict) else None


def _load_section_file(
    referrer: Path, section: tuple[str, ...], value: str, config_root: Path
) -> dict[str, Any]:
    name = ".".join(section)
    where = f"config {referrer} [{name}] from = {value!r}"
    path = _resolve(config_root, value)
    if not path.exists():
        raise ValueError(f"{where}: no file at {path}")
    with open(path, "rb") as handle:
        data: dict[str, Any] = tomllib.load(handle)
    if "extends" in data:
        raise ValueError(f"{where}: {path} sets `extends`, which a section file may not")
    if _contains_from_key(data):
        raise ValueError(f"{where}: {path} uses `from`, which a section file may not")
    table = _find_table(data, section)
    if table is None:
        raise ValueError(f"{where}: {path} has no [{name}]")
    return table


def _expand_section_files(
    table: dict[str, Any],
    section: tuple[str, ...],
    referrer: Path,
    config_root: Path,
    expanded: list[tuple[str, ...]],
) -> None:
    for key, value in table.items():
        if not isinstance(value, dict):
            continue
        path = (*section, key)
        if FROM_KEY not in value:
            _expand_section_files(value, path, referrer, config_root, expanded)
            continue
        where = f"config {referrer} [{'.'.join(path)}]"
        source = value[FROM_KEY]
        if not isinstance(source, str):
            raise ValueError(f"{where}: `from` must be a string")
        if len(value) != 1:
            raise ValueError(f"{where}: `from` must be the only key in the table")
        table[key] = _load_section_file(referrer, path, source, config_root)
        expanded.append(path)


def expand_section_files(
    data: dict[str, Any], referrer: Path, config_root: Path = CONFIG_ROOT
) -> list[tuple[str, ...]]:
    """Replace each ``from = "<file>"`` table in ``data`` with the file's table at that path."""
    if FROM_KEY in data:
        raise ValueError(f"config {referrer}: `from` is only valid inside a table")
    expanded: list[tuple[str, ...]] = []
    _expand_section_files(data, (), referrer, config_root, expanded)
    return expanded


def _replace_expanded_sections(
    merged: dict[str, Any], overlay: dict[str, Any], expanded: list[tuple[str, ...]]
) -> None:
    for section in expanded:
        parent = _find_table(merged, section[:-1])
        table = _find_table(overlay, section)
        if parent is not None and table is not None:
            parent[section[-1]] = table


def _load(path: Path, config_root: Path, chain: list[Path]) -> dict[str, Any]:
    canonical = path.resolve()
    if canonical in chain:
        raise ValueError(f"circular config inheritance: {[p.name for p in chain]} -> {path.name}")
    chain.append(canonical)
    with open(path, "rb") as handle:
        data: dict[str, Any] = tomllib.load(handle)
    expanded = expand_section_files(data, path, config_root)
    extends = data.pop("extends", None)
    if extends is None:
        return data
    merged = _load(_resolve(config_root, extends), config_root, chain)
    _merge(merged, data)
    _replace_expanded_sections(merged, data, expanded)
    return merged
