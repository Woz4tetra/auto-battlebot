#!/bin/bash
set -u
forbidden="${FORBIDDEN_NEEDED:-libnvjpeg|libauto_battlebot_nvjpg}"
status=0
for binary in "$@"; do
    if [ ! -f "$binary" ]; then
        echo "FAIL: $binary does not exist"
        status=1
        continue
    fi
    needed=$(readelf -d "$binary") || { echo "FAIL: readelf failed on $binary"; status=1; continue; }
    hits=$(grep -E "\(NEEDED\).*\[($forbidden)[^]]*\]" <<<"$needed")
    if [ -n "$hits" ]; then
        echo "FAIL: $binary links the nvjpeg plugin or libnvjpeg directly:"
        echo "$hits"
        status=1
    else
        echo "ok: $binary"
    fi
done
exit $status
