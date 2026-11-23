# C++ (src/ and include/)

Interfaces live in `include/<module>/`, implementations in `src/<module>/`. These rules cover both.

## Conventions

- New component: add interface to `include/<module>/`, implementation to `src/<module>/`, register in the factory
- Tunable values (thresholds, timeouts, rates, limits) go in the component's config struct as a
  defaulted field, not in a `static constexpr`, global, or file-scope constant. Keep constants for
  values fixed by math, a spec, or a file format (e.g. observation tensor sizes). Pass the value in
  through the constructor rather than reaching for a global
- Leave a config key out of the TOML files when its value matches the struct default. Write it
  only where a profile overrides it
- Config constants are enums, never strings. Any field with a fixed set of valid values gets an
  `enum class` in `include/enums/<name>.hpp`, an include in `include/enums.hpp`, and
  `PARSE_ENUM`/`PARSE_ENUM_REQUIRED` in the config struct. A typo then fails at parse with the
  valid values listed, instead of reaching a runtime string compare. Free-form strings are for
  paths, ids, and names only: `svo_file_path`, `hazards_file`, `active_profile`, label keys.
  The `type` field selecting a factory implementation is the one exception, since `REGISTER_CONFIG`
  keys on it
- Compiler flags are `-Wall -Wextra -Werror`. Fix warnings, do not suppress them
- A member shared between threads under a mutex uses `Mutex`, `MutexLock` and `GUARDED_BY` from
  `include/thread_safety/mutex.hpp`, not a bare `std::mutex`. Lint's clang-tidy pass enforces it
  (`docs/reference/tools/thread_safety.md`)
- Remote UI status topics and commands are added in `include/remote/protocol.hpp` (one row each,
  structs in `include/remote/messages.hpp`); `./scripts/lint` regenerates the web page's types
- Foxglove topics are rows of `kVizTopics` in `include/publisher/viz_topics.hpp`, never a string
  literal at the call site
- No blocking calls in the main perception loop. The latency budget is tight

## Formatting and lint

- Google style, 4-space indent, 100-char line limit (`.clang-format`)
- The clang toolchain is pinned to the major version in `.llvm-version`. The unversioned distro
  packages differ per release (14 on 22.04, 18 on 24.04, which is what JetPack 7 ships) and
  reformat the tree inconsistently. Every platform install sets it up; to repair it alone run
  `./install/install_llvm_toolchain.sh`. `scripts/lint` uses the versioned binaries and skips the
  step if they are absent
- `./scripts/lint` runs `clang-tidy` over `src/` and needs `build/` (`scripts/build.sh`)

## clang-tidy cache

The clang-tidy step is cached by [ctcache](https://github.com/matus-chochlik/ctcache),
pinned and installed by `install/install_clang_tidy_cache.sh` to
`~/.local/share/ctcache`. It hashes the preprocessed translation unit and replays
stored diagnostics on a hit, so a full run drops from ~50s to ~7s; comment- and
whitespace-only edits stay cache hits. Entries live in `~/.cache/ctcache`.
`scripts/lint` sets `CTCACHE_SAVE_OUTPUT=1`, so translation units that emit warnings are cached too
(without it only clean ones are, and anything warning re-runs every time). It sets `CTCACHE_DIR` to
`~/.cache/ctcache` because the default `/tmp/ctcache-$USER` is wiped by a reboot.

```bash
rm -rf ~/.cache/ctcache        # drop the cache
CTCACHE_DISABLE=1 ./scripts/lint  # one uncached run, to check the cache is honest
```

Lint still works without it installed, just uncached.
