#!/bin/bash
# update all submodules
# Assuming you are at the arm_teleop_ur5 directory level
git pull
git submodule update --init --recursive
src_path=$(pwd)
ur_sub_dir="/universal_robot"
teleop_keyboard_dir="/user_input/teleop_twist_keyboard"
teleop_joy_dir="/user_input/teleop_twist_joy"
vid_stm_cv_dir="/web_interface/video_stream_opencv"
cd "$src_path$ur_sub_dir"
echo "Updating: $(pwd)"
git checkout kinetic-devel
git pull
cd "$src_path$teleop_keyboard_dir"
echo "Updating: $(pwd)"
git checkout master
git pull
cd "$src_path$teleop_joy_dir"
git checkout indigo-devel
git pull origin indigo-devel
cd "$src_path$vid_stm_cv_dir"
echo "Updating: $(pwd)"
git checkout master
git pull origin master
cd "$src_path"
