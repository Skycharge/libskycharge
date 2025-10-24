#!/bin/bash
#
# Docker-optimized variant of build-for-arm.sh
#
# What this does (high level):
# - Mount the official BeagleBone Debian 9.4 (2018-06-17) image read-only
# - Make a writable copy of its rootfs via rsync (portable alternative to overlayfs)
# - Bind-mount proc/sys/dev/devpts/tmp and chroot into that copy using qemu-arm-static
# - Configure APT to use archive.debian.org (Stretch is EOL) and relax date/signature checks
# - Install build dependencies and run dpkg-buildpackage inside the chroot (ABI parity => libgps22)
# - Copy resulting Debian artifacts back to builds/skycharge/<version>
#
# Why not cross-compile?
# Cross-compilation previously produced ABI mismatches (e.g., libgps28). Building inside the
# original Stretch rootfs ensures the packages link against the exact target system libraries.
#
# For safety, prevent running as root unless explicitly allowed (the Docker runner sets this)
if [ "$EUID" -eq 0 ] && [ "${ALLOW_ROOT_IN_CONTAINER:-}" != "1" ]; then
	echo "Please run as normal user (or set ALLOW_ROOT_IN_CONTAINER=1 inside Docker)"
	exit
fi

if [ $# -lt 1 ]; then
	echo "Usage: <path to bone-debian-9.4-iot-armhf-2018-06-17-4gb.img>"
	exit
fi

# Repo root (parent of scripts directory)
# Repo root (parent of scripts directory)
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." >/dev/null 2>&1 && pwd )"

# Extract package version from debian/changelog: skycharge (X.Y.Z) stretch; urgency=...
VER=`head -1 $DIR/debian/changelog | awk -F'[()]' '{print $2}'`
# Optional: override version via env VERSION_OVERRIDE
if [ -n "${VERSION_OVERRIDE:-}" ]; then
    VER="$VERSION_OVERRIDE"
fi
# Place artifacts under builds/skycharge/<version> by default (override with PROJ_OVERRIDE)
PROJ="${PROJ_OVERRIDE:-skycharge}"

# Absolute output path under repo/builds
OUTDIR=`realpath -m $DIR/builds/$PROJ/$VER`

if [ -e $OUTDIR ]; then
    TS=$(date +%Y%m%d-%H%M%S)
    OUTDIR="${OUTDIR}-$TS"
    echo "WARNING: output dir exists. Using: $OUTDIR" >&2
fi


IMG=$1

# Check command exists
set -e
qemu-arm-static --version >> /dev/null
set +e

# Executed under sudo: mount image read-only, copy to writable rootfs dir
# NOTE: The BeagleBone image's rootfs partition commonly starts at sector 8192 (512-byte sectors).
# We offset the loop device by 8192<<9 (= 4 MiB) so we can mount the rootfs directly.
function prepare_rootfs()
{
	IMG=$1
	TMP=$(mktemp -d -t chroot-arm-image-XXXXXXXXXX)
	MNT=$TMP/mnt
	ROOT=$TMP/rootfs
	mkdir -p "$MNT" "$ROOT"

	set -e
	DEV=`losetup -r -o $((8192<<9)) --find --show $IMG`  # 8192*512 = 4 MiB offset to rootfs
	mount -o ro,noload $DEV $MNT
	# Copy the root filesystem to a writable directory; exclude special/volatile mounts
	rsync -a --delete --numeric-ids \
		--exclude=/dev/** --exclude=/proc/** --exclude=/sys/** \
		--exclude=/tmp/** --exclude=/run/** --exclude=/mnt/** --exclude=/media/** \
		$MNT/ $ROOT/
	umount $MNT
	losetup --detach $DEV
	set +e

	# Bind essential pseudo-filesystems required by tools inside the chroot
	mount -t proc proc $ROOT/proc
	mount -t sysfs sysfs $ROOT/sys
	mount -t devtmpfs devtmpfs $ROOT/dev
	mount -t devpts devpts $ROOT/dev/pts
	mount -t tmpfs tmpfs $ROOT/tmp

	echo "$TMP $ROOT"
}

function cleanup_rootfs()
{
	TMP=$1
	ROOT=$2
	umount $ROOT/tmp || true
	umount $ROOT/dev/pts || true
	umount $ROOT/dev || true
	umount $ROOT/sys || true
	umount $ROOT/proc || true
	rm -rf "$TMP"
}

PREP_FUNC=$(declare -f prepare_rootfs)
CLEAN_FUNC=$(declare -f cleanup_rootfs)

read TMPDIR BBB_DIR < <(sudo bash -c "$PREP_FUNC; prepare_rootfs $IMG")

# Prepare main folder under /work to avoid macOS Docker shared-volume quirks
# Copy the repository into the chroot to avoid cross-filesystem permission issues
sudo mkdir -p $BBB_DIR/work
rm -rf $BBB_DIR/work/$PROJ
cp -a $DIR $BBB_DIR/work/$PROJ

# If version override was provided, update the copied changelog inside chroot rootfs
# If VERSION_OVERRIDE is set, update the copied changelog so dpkg-buildpackage uses that version
if [ -n "${VERSION_OVERRIDE:-}" ]; then
	sudo bash -c "sed -i '1s/(\\([^)]*\\))/($VER)/' '$BBB_DIR/work/$PROJ/debian/changelog'"
fi

# Ensure qemu-arm-static exists in the chroot so ARM binaries can execute on the x86_64 host
if [ ! -e $BBB_DIR/usr/bin/qemu-arm-static ]; then
	sudo cp /usr/bin/qemu-arm-static $BBB_DIR/usr/bin/qemu-arm-static
fi

# Ensure DNS works inside chroot: replace dangling symlink with a static resolv.conf
sudo rm -f $BBB_DIR/etc/resolv.conf
printf "nameserver 1.1.1.1\nnameserver 8.8.8.8\n" | sudo tee $BBB_DIR/etc/resolv.conf >/dev/null

# Build packages under chroot. We:
# - configure apt for Stretch EOL archive and relax date/signature checks
# - install build deps (pin libserialport=0.1.1-1 to match BeagleBone era)
# - run dpkg-buildpackage to produce .deb artifacts
sudo PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
		 chroot $BBB_DIR qemu-arm-static /bin/bash -c "set -e; \
			 printf 'Acquire::Check-Valid-Until \"false\";\nAcquire::Check-Date \"false\";\nAcquire::AllowInsecureRepositories \"true\";\n' > /etc/apt/apt.conf.d/99archive || true; \
			 rm -f /etc/apt/sources.list.d/*.list || true; \
			 echo 'deb [trusted=yes] http://archive.debian.org/debian stretch main contrib non-free' > /etc/apt/sources.list; \
			 apt-get -o Acquire::Check-Valid-Until=false -o Acquire::Check-Date=false update; \
			 DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends --allow-downgrades \
				 build-essential dpkg-dev debhelper dh-autoreconf bison flex zlib1g-dev uuid-dev \
				 libzmq3-dev libczmq-dev libavahi-client-dev libmicrohttpd-dev \
				 libserialport0=0.1.1-1 libserialport-dev=0.1.1-1 \
				 libgps-dev libsystemd-dev libelf-dev pkg-config; \
			 cd /work/$PROJ; LC_ALL=en_US.UTF-8 dpkg-buildpackage -us -uc;"

mkdir -p $OUTDIR
# Copy only regular files (e.g., *_armhf.deb, .changes, .buildinfo) produced alongside /work/$PROJ
find $BBB_DIR/work -maxdepth 1 -type f -print0 | xargs -0 -I {} cp -a "{}" "$OUTDIR"

# Unmount and cleanup the temporary writable rootfs
sudo bash -c "$CLEAN_FUNC; cleanup_rootfs $TMPDIR $BBB_DIR"
