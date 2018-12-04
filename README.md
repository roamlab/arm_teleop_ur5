Arm Teleoperation - Universal Robot
==============================
## Prerequisite and dependencies
* Install ROS

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
