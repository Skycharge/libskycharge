#!/bin/bash
#
# Copyright (C) 2021-2022 Skycharge GmbH
# Author: Roman Penyaev <r.peniaev@gmail.com>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met: 1. Redistributions of source code must retain the above
# copyright notice, this list of conditions and the following
# disclaimer.
#
# 2. Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived
# from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
# INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
# OF THE POSSIBILITY OF SUCH DAMAGE.

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
PROJ=`basename $DIR`

# Abs path
OUTDIR=`realpath -m $DIR/../builds/$PROJ/$VER`

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

# Prepare main folder
rm -rf /mnt/tmp/$PROJ
# git clone $DIR /mnt/tmp/$PROJ
cp -a $DIR /mnt/tmp/$PROJ

if [ ! -e /mnt/usr/bin/qemu-arm-static ]; then
	sudo cp /usr/bin/qemu-arm-static /mnt/usr/bin/qemu-arm-static
fi

# Build packages under chroot
sudo PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
	 chroot /mnt qemu-arm-static /bin/bash -c "cd /tmp/$PROJ; LC_ALL=en_US.UTF-8 dpkg-buildpackage -us -uc;"

mkdir -p $OUTDIR
# Copy only regular files
find /mnt/tmp -maxdepth 1 -type f | xargs -I {} cp {} $OUTDIR

# Umount
sudo bash -c "$UMOUNT_FUNC; umount_image $DEV"
