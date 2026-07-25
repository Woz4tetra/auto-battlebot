# Writes the short commit hash to a text file next to the binary, suffixed with
# "-dirty" when the working tree has uncommitted changes to tracked files. Run
# via `cmake -P` from a custom target so it re-evaluates on every build and the
# dirty flag stays accurate. The binary reads the file at startup, so nothing
# recompiles or relinks when the hash changes.
#
# On machines with a git repo (dev hosts) the version comes from git directly.
# On machines without one (the Jetson, where deploy_to_jetson.sh excludes .git)
# it falls back to the .build_version file baked in at deploy time.
#
# Inputs (passed with -D):
#   GIT_SOURCE_DIR - repository working directory
#   GIT_OUTPUT     - path of the text file to write

find_package(Git QUIET)

set(_git_version "")

if(Git_FOUND)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
        WORKING_DIRECTORY ${GIT_SOURCE_DIR}
        OUTPUT_VARIABLE _git_hash
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        RESULT_VARIABLE _hash_result
    )

    if(_hash_result EQUAL 0 AND NOT _git_hash STREQUAL "")
        set(_git_version "${_git_hash}")

        # --untracked-files=no: only tracked modifications count as dirty,
        # matching the conventional `-dirty` suffix semantics.
        execute_process(
            COMMAND ${GIT_EXECUTABLE} status --porcelain --untracked-files=no
            WORKING_DIRECTORY ${GIT_SOURCE_DIR}
            OUTPUT_VARIABLE _git_status
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET
        )
        if(NOT _git_status STREQUAL "")
            set(_git_version "${_git_version}-dirty")
        endif()
    endif()
endif()

# Fallback for no-repo machines: read the version stamped in at deploy time.
if(_git_version STREQUAL "" AND EXISTS "${GIT_SOURCE_DIR}/.build_version")
    file(READ "${GIT_SOURCE_DIR}/.build_version" _git_version)
    string(STRIP "${_git_version}" _git_version)
endif()

if(_git_version STREQUAL "")
    set(_git_version "unknown")
endif()

set(_content "${_git_version}\n")

# Only rewrite when the value changed, so the file's mtime stays meaningful.
set(_existing "")
if(EXISTS ${GIT_OUTPUT})
    file(READ ${GIT_OUTPUT} _existing)
endif()
if(NOT _existing STREQUAL _content)
    file(WRITE ${GIT_OUTPUT} "${_content}")
endif()
