
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

if(_git_version STREQUAL "" AND EXISTS "${GIT_SOURCE_DIR}/.build_version")
    file(READ "${GIT_SOURCE_DIR}/.build_version" _git_version)
    string(STRIP "${_git_version}" _git_version)
endif()

if(_git_version STREQUAL "")
    set(_git_version "unknown")
endif()

set(_content "${_git_version}\n")

set(_existing "")
if(EXISTS ${GIT_OUTPUT})
    file(READ ${GIT_OUTPUT} _existing)
endif()
if(NOT _existing STREQUAL _content)
    file(WRITE ${GIT_OUTPUT} "${_content}")
endif()
