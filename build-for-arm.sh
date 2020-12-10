#!/bin/bash

if [ "$EUID" -eq 0 ]; then
	echo "Please run as normal user"
	exit
fi

if [ $# != 1 ]; then
	echo "Usage: <path to bone-debian-9.4-iot-armhf-2018-06-17-4gb.img>"
	exit
fi

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
VER=`head -1 $DIR/debian/changelog | awk -F'[()]' '{print $2}'`

# Abs path
OUTDIR=`readlink -f $DIR/../builds/$VER`

if [ -e $OUTDIR ]; then
	printf "ERROR: output build directory '%s' exists.\n" $OUTDIR
	exit
fi


IMG=$1

# Check command exists
set -e
qemu-arm-static --version >> /dev/null
set +e

# Executed under sudo
function mount_image()
{
	IMG=$1

	set -e
	DEV=`losetup -o $((8192<<9)) --find --show $IMG`
	set +e

	mount $DEV /mnt
	mount -t proc proc /mnt/proc
	mount -t sysfs sysfs /mnt/sys
	mount -t devtmpfs devtmpfs /mnt/dev
	mount -t tmpfs tmpfs /mnt/tmp

	echo $DEV
}

# Executed under sudo
function umount_image()
{
	DEV=$1

	umount /mnt/tmp
	umount /mnt/dev
	umount /mnt/sys
	umount /mnt/proc
	umount /mnt
	losetup --detach $DEV
}

MOUNT_FUNC=$(declare -f mount_image)
UMOUNT_FUNC=$(declare -f umount_image)

# Mount everything we need for chrooting
DEV=`sudo bash -c "$MOUNT_FUNC; mount_image $IMG"`

# Prepare main libskysense folder
rm -rf /mnt/tmp/libskysense
# git clone $DIR /mnt/tmp/libskysense
cp -a $DIR /mnt/tmp/libskysense

# Build packages under chroot
sudo PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
	chroot /mnt qemu-arm-static /bin/bash -c 'cd /tmp/libskysense; LC_ALL=en_US.UTF-8 dpkg-buildpackage -us -uc;'

mkdir -p $OUTDIR
# Copy only regular files
find /mnt/tmp -maxdepth 1 -type f | xargs -I {} cp {} $OUTDIR

# Umount
sudo bash -c "$UMOUNT_FUNC; umount_image $DEV"
