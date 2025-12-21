#!/usr/bin/env bash
set -euo pipefail

source "/opt/ros/kilted/setup.bash"

if [ -f "/ws/install/setup.bash" ]; then
  source "/ws/install/setup.bash"
fi

exec "$@"
