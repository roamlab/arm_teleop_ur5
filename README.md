Arm Teleoperation - Universal Robot
==============================
## Prerequisite and dependencies
* Install [ROS](http://wiki.ros.org/ROS/Installation)
* Install [ROS-Industrial](http://wiki.ros.org/Industrial/Install)
* Install [MoveIt](https://moveit.ros.org/install/) `sudo apt-get install ros-kinetic-moveit`
* Downlaod and build [cisst-saw](https://github.com/jhu-cisst/cisst/wiki/Compiling-cisst-and-SAW-with-CMake#13-building-using-catkin-build-tools-for-ros), using **catkin build tools** in a **seperate** workspace.  
**Note:** In this application, we use the **[sawUniversalRobot](https://github.com/jhu-saw/sawUniversalRobot/tree/devel)** component to establish an interface that we can set velocities at 125 Hz.

## [Optional] Debuggin interface and tools
* Install iPython  
`sudo apt-get install ipython`
* Install `ipdb` for setting break point in command line ipython  
`sudo apt-get install python-ipdb`

## Downlaod & Compiling 
We are using `catkin_make` for this repo, and first, let's set up the workspace:    
* `mkdir -p catkin_ws/src`  
* `cd catkin_ws`  
* `catkin_make`  

Then, let's get the code (if this is the first time you download this repo):
* `cd src`  
* `git clone git@github.com:roamlab/arm_teleop_ur5.git --recursive`  
* `cd arm_teleop_ur5`  
* `bash update_repo.sh`  

Finally, to compile, since this repo depends on another catkin workspace, we need to [overlay workspaces](http://wiki.ros.org/catkin/Tutorials/workspace_overlaying)
* `cd catkin_ws`  
* `rm -rf build` & `rm -rf devel`  
* IMPORTANT `source <cisst_catkin_ws/devel_release/setup.bash>`  
* `catkin_make`  
Note that if the first time `catkin_make` failed, try it again, it should succeed in the second time.

## Running Instructions
Before running the code, you need to source and define env var:  
`source <teleop_catkin_workspace>/devel/setup.bash`  
`export ARM_TELEOP_UR5_SRC="<arm_teleop_ur5/src>"`  
You may also choose to copy them at the end of your `~/.bashrc` file
