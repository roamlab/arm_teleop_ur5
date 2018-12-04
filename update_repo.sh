#!/bin/bash
# update all submodules
# Assuming you are at the arm_teleop_ur5 directory level
git pull
src_path=$(pwd)
ur_sub_dir="/universal_robot"
cd "$src_path$ur_sub_dir"
echo "Updating: $(pwd)"
git checkout kinetic-devel
git pull
git submodule update --init --recursive
cd "$src_path"