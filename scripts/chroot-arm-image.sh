#!/bin/bash

if [ "$EUID" -eq 0 ]; then
	echo "Please run as normal user"
	exit
fi

if [ $# == 0 ]; then
	echo "Usage: <path to bone-debian-9.4-iot-armhf-2018-06-17-4gb.img> [--ro]"
	exit
fi

BBB_IMAGE=$1
RO=0
if [ "$2" == "--ro" ]; then
	RO=1
fi

# Check commands exist
set -e
qemu-arm-static --version >> /dev/null
set +e

tmp_dir=$(mktemp -d -t chroot-arm-image-XXXXXXXXXX)

if [ "$RO" == "1" ]; then
	OL_LOWER_DIR=$tmp_dir/overlay/bbb.image.ro
	OL_UPPER_DIR=$tmp_dir/overlay/upper
	OL_WORK_DIR=$tmp_dir/overlay/work
	BBB_DIR=$tmp_dir/bbb.image.rw

	mkdir -p $BBB_DIR
	mkdir -p $OL_LOWER_DIR
	mkdir -p $OL_UPPER_DIR
	mkdir -p $OL_WORK_DIR
else
	BBB_DIR=$tmp_dir
fi

# Executed under sudo
function mount_image()
{
	BBB_IMAGE=$1
	BBB_DIR=$2
	RO=$3
	OL_LOWER_DIR=$4
	OL_UPPER_DIR=$5
	OL_WORK_DIR=$6

	ro_flag=""
	if [ "$RO" == "1" ]; then
		ro_flag="-r"
	fi

	set -e

	LODEV=`losetup $ro_flag -o $((8192<<9)) --find --show $BBB_IMAGE`
	if [ "$RO" == "1" ]; then
		mount -o ro,noload $LODEV $OL_LOWER_DIR
		mount -t overlay overlay -o lowerdir=$OL_LOWER_DIR,upperdir=$OL_UPPER_DIR,workdir=$OL_WORK_DIR $BBB_DIR
	else
		mount $LODEV $BBB_DIR
	fi
	mount -t proc proc $BBB_DIR/proc
	mount -t sysfs sysfs $BBB_DIR/sys
	mount -t devtmpfs devtmpfs $BBB_DIR/dev
	mount -t devpts devpts $BBB_DIR/dev/pts
	# Keep /run and /run/user in order to make ssh-agent work
	mount --bind /run $BBB_DIR/run
	mount --bind /run/user/$SUDO_UID $BBB_DIR/run/user/$SUDO_UID

	losetup --detach $LODEV
}

# Executed under sudo
function umount_image()
{
	BBB_DIR=$1
	RO=$2
	OL_LOWER_DIR=$3

	umount $BBB_DIR/run/user/$SUDO_UID
	umount $BBB_DIR/run
	umount $BBB_DIR/dev/pts
	umount $BBB_DIR/dev
	umount $BBB_DIR/sys
	umount $BBB_DIR/proc
	umount $BBB_DIR
	if [ "$RO" == "1" ]; then
		umount $OL_LOWER_DIR
	fi
}

MOUNT_FUNC=$(declare -f mount_image)
UMOUNT_FUNC=$(declare -f umount_image)

# Mount everything we need for chrooting
sudo bash -c "$MOUNT_FUNC; mount_image $BBB_IMAGE $BBB_DIR $RO $OL_LOWER_DIR $OL_UPPER_DIR $OL_WORK_DIR"

# Copy qemu-arm-static to bbb image
if [ ! -e $BBB_DIR/usr/bin/qemu-arm-static ]; then
	sudo cp /usr/bin/qemu-arm-static $BBB_DIR/usr/bin/qemu-arm-static
fi

# Chroot, preserve env like SSH_SOCK_AUTH to make ssh-agent work
sudo --preserve-env PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
	 chroot $BBB_DIR qemu-arm-static /bin/bash

# Finish
sudo bash -c "$UMOUNT_FUNC; umount_image $BBB_DIR $RO $OL_LOWER_DIR"
sudo rm -r $tmp_dir
