#!/bin/bash
# update all submodules
# Assuming you are at the arm_teleop_ur5 directory level
git pull
git submodule update --init --recursive
src_path=$(pwd)
ur_sub_dir="/universal_robot"
teleop_keyboard_dir="/user_input/teleop_twist_keyboard"
cd "$src_path$ur_sub_dir"
echo "Updating: $(pwd)"
git checkout kinetic-devel
git pull
cd "$src_path$teleop_keyboard_dir"
echo "Updating: $(pwd)"
git checkout master
git pull
cd "$src_path"