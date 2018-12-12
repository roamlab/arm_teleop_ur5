Configuration Files for Teleoperation
============================================
This folder contains the configuration files for teleoperation.

# user_interface
* **name** - the name of the user interface device, could be keyboard, or a joy game pad.
* **TF_device2view_XXX** - information related to the homogenous transformation matrix from the device to the user's view.  
[**IMPORTANT NOTE**] In this repo, we refer to the definition of a TF frame from frame {**A**} to frame {**B**} as:  
vector_in_B = TF_A2B * vector_in_A
* **TF_device2view_rot_X** - the **x** axis in the rotation matrix portion, which corresponds to the first column
* **TF_device2view_rot_Y**
* **TF_device2view_rot_Z**
* **TF_device2view_trans** - the translation portion (origin position) of the homogenous transformation.
