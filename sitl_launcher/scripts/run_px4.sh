#!/usr/bin/env sh

dir=`mktemp -d`
echo "temporary rootfs: $dir"
# cp -R mixers $dir
# cp -R rootfs $dir

cp -R /home/tfoote/work/ar/ws/install/share/px4/* $dir

cd $dir
rc_script=$1
px4 ${dir} ${rc_script}
