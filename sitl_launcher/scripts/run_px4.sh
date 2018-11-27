#!/usr/bin/env sh

set -e

# Argument 1 rc_script
# Argument 2 px4_sim_model

tempdir=`mktemp -d`
echo "temporary rootfs: $tempdir"

px4dir=$(rospack find px4)
if [ -z $px4dir ]; then
    echo "px4 dir not detected!"
    exit 1 
fi
echo "px4 dir detected: $px4dir"

cp -R ${px4dir}/* $tempdir

export PX4_SIM_MODEL=$2

cd $tempdir
rc_script=$1
echo About to run: px4 ${tempdir}/ROMFS/px4fmu_common -s ${rc_script} -d -t /home/osrf/src/Firmware/test_data
px4 ${tempdir}/ROMFS/px4fmu_common -s ${rc_script} -d