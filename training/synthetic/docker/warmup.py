import blenderproc as bproc  # isort: skip  # must be first import for blenderproc run

# Docker-build warmup that primes BlenderProc's installed-packages cache.
#
# `blenderproc pip install <pkg>` records only the named package
# (install_default_packages=False), so on its own the cache never lists
# BlenderProc's default runtime packages (pyyaml, imageio, gitpython, ...).  That
# makes every runtime `blenderproc run` re-run pip to "check" them (the
# "Installing pip package ... Requirement already satisfied" spam).
#
# Running `blenderproc run` once here goes through `setup_pip` with
# install_default_packages=True, which installs/records the defaults into
# <blender>/custom-python-packages/installed_packages_cache_v2.json.  Baking a
# complete cache into the image makes runtime `blenderproc run` skip the pip
# re-check entirely.  The build step deletes any stale cache first so the fresh,
# complete cache is always written.

bproc.init()
print("[warmup] BlenderProc package cache primed")
