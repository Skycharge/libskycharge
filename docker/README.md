# BeagleBone image-based build (Docker)

Build Debian packages inside the original BeagleBone Debian 9.4 (2018-06-17) image for exact ABI matching (e.g., libgps22). No cross-compiling: we chroot into the image using qemu.

## Prerequisites
- Download and uncompress the IoT image into this folder:
	- https://files.beagle.cc/file/beagleboard-public-2021/images/bone-debian-9.4-iot-armhf-2018-06-17-4gb.img.xz
	- Result: `docker/bone-debian-9.4-iot-armhf-2018-06-17-4gb.img`

## Quick start
```bash
cd docker
./build-bone.sh                 # uses bone-debian-9.4-...img by default
# or: ./build-bone.sh /absolute/path/to/image.img
# optional: VERSION_OVERRIDE=2.2.3 ./build-bone.sh
```
Artifacts go to `../builds/skycharge/<version>/`.

## Install on BeagleBone
```bash
# copy your .debs
scp ../builds/skycharge/*/*.deb root@skydevice.local:updates
# install (auto-fix deps)
ssh root@skydevice.local "cd /tmp && dpkg -i *.deb || apt-get -f install -y"
```
Note: Debian Stretch is EOL. If apt fails on the device, switch sources to archive.debian.org and allow unsigned/expired metadata.

## Clean
```bash
cd docker
./clean.sh         # remove builds/
./clean.sh --deep  # also prune runner image and Docker caches
```

## Files
- `Dockerfile.bone-runner` – helper container with qemu + mount tools
- `build-bone.sh` – builds the runner and executes the chrooted build
- `inside-bone-build.sh` – calls `scripts/build-for-arm-docker.sh` in the container
- `clean.sh` – cleanup helper
