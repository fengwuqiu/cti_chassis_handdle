#!/usr/bin/env bash
set -e

CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"

PKGS=(
    cti_fpga_data
    cti_fpga_serial
    cti_fpga_serial_msg
    mcu_ota
    cti_fpga #meta
)

for pkg in "${PKGS[@]}"; do
    cd ${CURR_DIR}/${pkg}
    bash .make_deb.sh
done
