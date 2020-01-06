#!/bin/bash

tab=" --tab"
options=(--tab)

cmds[1]="rosrun universal_robot_ros universal_robot -i 192.168.1.202"
titles[1]="robot_ip"

cmds[2]="roslaunch ur_description ur5_upload.launch"
titles[2]="ur5_upload"

cmds[3]="roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch"
titles[3]="ur5_moveit_planning_execution"

cmds[4]="rosrun teleop teleop_main.py joy_base_frame.cfg"
titles[4]="teleop_main"

cmds[5]="rosrun teleop_twist_keyboard teleop_twist_keyboard.py"
titles[5]="teleop_twist_keyboard"

cmds[6]="roslaunch teleop ur5_rvis.launch"
titles[6]="ur5_rvis"

for i in 1 2 3 4 5 6 ; do
  options+=($tab -e "bash -c \"${cmds[i]} ; bash\"" )
done

gnome-terminal "${options[@]}"


exit 0