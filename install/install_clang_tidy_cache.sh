# ctcache caches clang-tidy results, keyed on a hash of the preprocessed
# translation unit. A full scripts/lint run re-analyses all of src/ from scratch
# and takes ~48s; with a warm cache an unchanged run is ~11s.
#
# Upstream ships the wrapper we point run-clang-tidy at: a `clang-tidy` script at
# the repo root that reads the real binary from CTCACHE_CLANG_TIDY and execs the
# Python client beside it. The client is stdlib-only, so nothing is installed
# into the venv; requirements.txt there covers the optional server mode, which we
# do not use.
#
# Pinned to a commit rather than a tag: ctcache does not cut releases, and the
# hash it computes is part of our cache key, so an unpinned upgrade would quietly
# invalidate every entry.
CTCACHE_COMMIT="0c7fee26e09ae8a393ed7d56164102da118ab23a"
CTCACHE_REPO="https://github.com/matus-chochlik/ctcache.git"

install_clang_tidy_cache() {
    local install_dir="${HOME}/.local/share/ctcache"
    local wrapper="${install_dir}/clang-tidy"

    if [ -x "$wrapper" ] &&
        [ "$(git -C "$install_dir" rev-parse HEAD 2>/dev/null)" = "$CTCACHE_COMMIT" ]; then
        echo "clang-tidy cache already installed: ${install_dir} @ ${CTCACHE_COMMIT:0:12}"
        return 0
    fi

    if ! command -v git &>/dev/null; then
        echo "Cannot install clang-tidy cache: git not found"
        return 1
    fi

    if [ ! -d "$install_dir/.git" ]; then
        echo "Installing clang-tidy cache to ${install_dir}..."
        rm -rf "$install_dir"
        mkdir -p "$(dirname "$install_dir")"
        git clone --quiet "$CTCACHE_REPO" "$install_dir"
    else
        echo "Updating clang-tidy cache in ${install_dir}..."
        git -C "$install_dir" fetch --quiet origin
    fi

    git -C "$install_dir" checkout --quiet "$CTCACHE_COMMIT"
    chmod +x "$wrapper"
    echo "clang-tidy cache installed: ${install_dir} @ ${CTCACHE_COMMIT:0:12}"
}
