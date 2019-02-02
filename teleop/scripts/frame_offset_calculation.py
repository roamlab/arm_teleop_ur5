#!/usr/bin/env python
from PyKDL import *
import numpy as np
import ipdb

if __name__ == '__main__':
	R = Rotation(
		0, -1, 0,
		-1, 0, 0,
		0, 0, -1)
	theta = np.pi/180*(-45)
	R_t = Rotation.RotY(theta)
	R_res = R*R_t
	print("TF_view2robot_rot_X="),	
	print(R_res.UnitX())
	print("TF_view2robot_rot_Y="),	
	print(R_res.UnitY())
	print("TF_view2robot_rot_Z="),	
	print(R_res.UnitZ())

	# ipdb.set_trace()
