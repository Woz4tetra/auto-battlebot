#!/bin/bash
# Source first from every install entry point. Any failing command then prints its file, line,
# and text before the script exits, instead of `set -e` dropping back to the prompt in silence.
#
# -E carries the ERR trap into functions, sourced files, and command substitutions. The ERR trap
# only records the location; the EXIT trap prints it, since EXIT fires on every exit path.
# Install functions must not set their own traps or change shell options for the caller:
# `scripts/lint` rejects both (see its "install scripts" step).

set -eE
_install_fail_where=""
trap '_install_fail_where="${BASH_SOURCE[0]}:${LINENO}: ${BASH_COMMAND}"' ERR
trap '_install_rc=$?; if [ "$_install_rc" -ne 0 ]; then
    echo "INSTALL FAILED (exit $_install_rc) at ${_install_fail_where:-unknown location}" >&2
fi' EXIT
