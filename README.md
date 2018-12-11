Arm Teleoperation - Universal Robot
==============================
## Prerequisite and dependencies
* Install [ROS](http://wiki.ros.org/ROS/Installation)
* Install [ROS-Industrial](http://wiki.ros.org/Industrial/Install)
* Downlaod and build [cisst-saw](https://github.com/jhu-cisst/cisst/wiki/Compiling-cisst-and-SAW-with-CMake#13-building-using-catkin-build-tools-for-ros), using **catkin build tools** in a **seperate** workspace.  
**Note:** In this application, we use the **sawUniversalRobot** component to establish an interface that we can set velocities at 125 Hz.

## Install & Compiling 
We are using `catkin_make` for this repo, and first, let's set up the workspace:    
* `mkdir -p catkin_ws/src`  
* `cd catkin_ws`  
* `catkin_make`  

Then, let's get the code (if this is the first time you download this repo):
* `cd src`  
* `git clone git@github.com:roamlab/arm_teleop_ur5.git --recursive`  
* `cd arm_teleop_ur5`  
* `bash update_repo.sh`  
* `cd ../..`  
* `catkin_make`  Note that if the first time `catkin_make` failed, try it again, it should succeed in the second time.
