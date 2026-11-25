# tools

Standalone non-Python tools go in `tools/<name>/`, one directory per tool with its own build file
and its tests inside: Rust (`Cargo.toml`), Go (`go.mod`), C++, or a TypeScript frontend that needs
a build step (`package.json`).

- A built frontend that a Python module serves writes its output into that module's `static/`; the
  TypeScript source stays in `tools/`
- `web/` stays the dashboard
- `scripts/verify` has no suite for `tools/` and `scripts/lint` does not cover it. The first tool
  with its own build file adds its suite to `scripts/verify` and its lint step
- Shell entry points stay at the top level of `scripts/`
