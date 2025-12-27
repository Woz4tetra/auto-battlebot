#!/bin/bash

install_sam3() {
    set +e

    local install_dir="${1:-${HOME}/.local/facebookresearch}"

    # Check if SAM3 is already installed
    python -c "import sam3"
    IS_SAM3_INSTALLED=$?
    if [ "${IS_SAM3_INSTALLED}" -eq 0 ]; then
        echo "SAM3 is already installed at $install_dir"
        return 0
    fi

    set -e

    mkdir -p "${install_dir}"
    cd "${install_dir}"

    if [ ! -d "${install_dir}/sam3" ]; then
        git clone https://github.com/facebookresearch/sam3.git
    fi
    cd sam3

    pip install -e .
    pip install -e ".[notebooks]"
    pip install 'numpy>=2.0.0'  # reinstall numpy 2
}
