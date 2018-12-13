Configuration Files for Teleoperation
============================================
This folder contains the configuration files for teleoperation.
[**IMPORTANT NOTE**] In this repo, we refer to the definition of a TF frame from frame {**A**} to frame {**B**} as:  
vector_in_B = TF_A2B * vector_in_A

## user_interface
* **name** - the name of the user interface device, could be keyboard, or a joy game pad.
* **TF_device2view_XXX** - a frame representing the homogenous transformation from the device to the user's view.  
* **TF_device2view_rot_X** - the **x** axis in the rotation matrix portion, which corresponds to the first column
* **TF_device2view_rot_Y** - the **y** axis ...
* **TF_device2view_rot_Z** - the **z** axis ...
* **TF_device2view_trans** - the translation portion (origin position) of the homogenous transformation.
* **command_type** - can either be twist or pose, which corresponds to sending combined linear/angular velocities or a relative pose
* **rostopic_manipulator_comd** - rostopic to listen on, receiving the user interface input for commanding the manipulator motion 
* **rostopic_end_effector_comd** - rostopic to listen on, receiving the user interface input for commanding the end-effector actions, e.g. robotic hand open/close.

## manipulator
* **name** - the manipulator being used, for now limited to UR5
* **TF_view2robot_XXX** - a frame representing the homogenous transformation from the the user's view to the robot frame (the robot frame could mean either the base frame or the tool frame according to the **command_reference_frame**)
* **TF_view2robot_rot_X** - the **x** axis ...
* **TF_view2robot_rot_Y** - the **x** axis ...
* **TF_view2robot_rot_Z** - the **x** axis ...
* **TF_view2robot_trans** - the translation portion (origin position) of the homogenous transformation.
* **command_type** - can either be twist or pose, which corresponds to sending combined linear/angular velocities or a relative pose
* **command_reference_frame** - can either be **end_effector_frame** or **base_frame**, where **end_effector_frame** is more desirable if a camera is mounted on the end-effector.
* **rostopic_pose_current** - rostopic to listen on, fetching the current pose (cartesian position and orientaion) 
* **rostopic_joint_current** - rostopic to listen on, fetching the current joint states (joint angles, joint velocities, joint effort if available)
* **rostopic_pose_set** - rostopic to publish on, writing the desired commanding robot pose (position and orientation)
* **rostopic_twist_set** - rostopic to publish on, writing the desired commanding twist (linear and angular velocities) 

## end_effector
* **name** - name of the end-effector
* **rostopic_command** - rostopci to publish on, writing the desired commanding action for the end-effector
