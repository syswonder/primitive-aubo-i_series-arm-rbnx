#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
# Lifecycle Driver stubs. The ROS message types used here (sensor_msgs,
# geometry_msgs) are the stock ROS 2 packages from the runtime image.
rbnx codegen -p "$PKG"
python3 -m compileall -q "$PKG/aubo_arm"
