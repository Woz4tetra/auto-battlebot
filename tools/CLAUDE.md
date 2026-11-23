# tools

Standalone non-Python tools go in `tools/<name>/`, one directory per tool with its own build file
and its tests inside: Rust (`Cargo.toml`), Go (`go.mod`), C++, or a TypeScript frontend that needs
a build step (`package.json`).

- A built frontend that a Python module serves writes its output into that module's `static/`
  (`tools/feature_map` builds into `auto_battlebot/features/static/`); the TypeScript source stays
  in `tools/`
- `web/` stays the dashboard
- `scripts/verify` (`tools` suite) and `scripts/lint` (prettier) cover TypeScript tools with a
  `package.json`. The first tool in another language adds its suite and lint step
- Shell entry points stay at the top level of `scripts/`
