#!/usr/bin/env bash
# Thin wrapper that runs inside the bone-runner container. It simply invokes the
# docker-optimized build script with the mounted BeagleBone image. The heavy
# lifting happens in scripts/build-for-arm-docker.sh.
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: inside-bone-build.sh /path/to/bone-debian-9.4-iot-armhf-2018-06-17-4gb.img"
  echo "Environment: VERSION_OVERRIDE=2.2.3 to force package version"
  exit 2
fi

IMG=$1  # path to the mounted BeagleBone .img inside the container

# Reuse existing scripts with minimal changes. In the container, the project is mounted at /workspace
SCRIPTS_DIR="/workspace/scripts"

if [[ ! -f "$SCRIPTS_DIR/build-for-arm.sh" ]]; then
  echo "ERROR: scripts/build-for-arm.sh not found" >&2
  exit 2
fi

# We just call the existing script from inside the container; it expects host tools present.
# Allow running as root inside the privileged container
export ALLOW_ROOT_IN_CONTAINER=1
exec bash "$SCRIPTS_DIR/build-for-arm-docker.sh" "$IMG"
