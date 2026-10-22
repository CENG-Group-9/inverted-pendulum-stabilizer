#!/bin/bash
# Configure and build the GUI with the presets in this directory (CMakePresets.json).
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"

cmake --preset debug
cmake --build --preset debug
