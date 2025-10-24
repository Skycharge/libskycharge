#!/usr/bin/env bash
# Entry point for building Debian packages inside the BeagleBone image using the
# helper runner container and qemu+chroot. This script:
#  - builds the minimal runner image (Dockerfile.bone-runner)
#  - runs the container privileged with the project and .img mounted
#  - calls inside-bone-build.sh inside the container, which invokes scripts/build-for-arm-docker.sh
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$HERE/.." && pwd)"
IMG_DEFAULT="$HERE/bone-debian-9.4-iot-armhf-2018-06-17-4gb.img"

# Optional first argument overrides the default image path under docker/
IMG_PATH="${1:-$IMG_DEFAULT}"

if [[ ! -f "$IMG_PATH" ]]; then
  echo "ERROR: BeagleBone image not found: $IMG_PATH" >&2
  echo "Place the image at: $IMG_DEFAULT or pass a path: $0 /path/to/image.img" >&2
  exit 2
fi

# Build the helper container (has qemu-user-static and mount tools)
echo "Building bone-runner image..."
docker build -t bone-runner -f "$HERE/Dockerfile.bone-runner" "$HERE"

# Run the chrooted build. VERSION_OVERRIDE, if exported, is passed through.
echo "Running chrooted build inside: $IMG_PATH"
docker run --rm -it \
  --privileged \
  -v "$REPO_ROOT:/workspace" \
  -v "$IMG_PATH:/img.img:ro" \
  -e VERSION_OVERRIDE \
  bone-runner bash -lc \
  "/usr/local/bin/inside-bone-build.sh /img.img"

echo "Build finished. Artifacts should be under: $REPO_ROOT/builds/skycharge" 