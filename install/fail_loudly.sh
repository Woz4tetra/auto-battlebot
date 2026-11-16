#!/bin/bash

set -eE
_install_fail_where=""
trap '_install_fail_where="${BASH_SOURCE[0]}:${LINENO}: ${BASH_COMMAND}"' ERR
trap '_install_rc=$?; if [ "$_install_rc" -ne 0 ]; then
    echo "INSTALL FAILED (exit $_install_rc) at ${_install_fail_where:-unknown location}" >&2
fi' EXIT
