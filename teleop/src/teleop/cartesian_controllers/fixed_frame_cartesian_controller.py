#!/usr/bin/env python
import numpy as np
import PyKDL
from tf_conversions import posemath as PoseMath
from teleop.cartesian_controllers import CartesianControllerBaseClass
from teleop.utils.transform_helpers import convert_PyKDL_twist_to_geometry_msgs_twist
from teleop.utils.transform_helpers import convert_twist_to_numpy_array
from teleop.utils import ResolvedRatesCalculator


class FixedFrameCartesianController(CartesianControllerBaseClass):
    def __init__(self, config_data, section_name):
        super(FixedFrameCartesianController, self).__init__(config_data, section_name)
        resolved_rates_section_name = config_data.get(section_name, 'resolved_rates')
        self.resolved_rates_computer = ResolvedRatesCalculator(config_data, resolved_rates_section_name)

    def compute_vee(self, current_pose, desired_pose):
        """

        Args:
            current: Pose()
            desired_pydkl_frame: Pose()

        Returns:
           a numpy array of the resulting end effector velocity given in the fixed frame
        """
        # compute pose error (result in kdl.twist format)
        current_pydkl_frame = PoseMath.fromMsg(current_pose)
        desired_pydkl_frame = PoseMath.fromMsg(desired_pose)
        pose_error_pydkl = PyKDL.diff(current_pydkl_frame, desired_pydkl_frame)

        pose_error_geo = convert_PyKDL_twist_to_geometry_msgs_twist(pose_error_pydkl)
        pose_error_numpy = convert_twist_to_numpy_array(pose_error_geo)
        vel_mag, ang_vel_mag = self.resolved_rates_computer.resolve_rates(pose_error_numpy)

        desired_pykdl_twist = PyKDL.Twist()
        pose_error_pydkl.vel.Normalize()  # normalize to have the velocity direction
        desired_pykdl_twist.vel = pose_error_pydkl.vel * vel_mag
        pose_error_pydkl.rot.Normalize()  # normalize to have the ang vel direction
        desired_pykdl_twist.rot = pose_error_pydkl.rot * ang_vel_mag

        desired_twist = convert_PyKDL_twist_to_geometry_msgs_twist(desired_pykdl_twist)
        numpy_twist = convert_twist_to_numpy_array(desired_twist)
        return numpy_twist

    def compute_qd(self, v_ee, joint_transforms, fixed_T_tool_current, joint_axes=None):
        """

        Args:
            v_ee: np.array velocity of the end effector given in the fixed (world/base) frame
            joint_transforms:
            fixed_T_tool_current:
            joint_axes:

        Returns:

        """
        # assemble Jacobian
        J = self.assemble_J_with_any_axis_joints(joint_transforms, fixed_T_tool_current, joint_axes)
        # compute Jacobian pseudo-inverse, discarding motion for SV's below a threshold
        Jp = np.linalg.pinv(J, 1.0e-2)
        # multiply by desired end-effector velocity to get joint velocities
        dq = np.dot(Jp, v_ee)
        # scale joint velocities down to be below limit
        max_dq = 0
        for i in range(0, len(dq)):
            if abs(dq[i]) > max_dq:
                max_dq = abs(dq[i])
        if max_dq > 1.0:
            for i in range(0, len(dq)):
                dq[i] = dq[i] / max_dq
        return dq

    def assemble_J_with_any_axis_joints(self, joint_transforms, fixed_T_tool_current, joint_axes):
        J = np.zeros((6, len(joint_transforms)))
        o_n = fixed_T_tool_current[0:3, 3]
        for j in range(len(joint_transforms)):
            fixed_T_j = joint_transforms[j]
            fixed_R_j = fixed_T_j[0:3, 0:3]
            fixed_t_j = fixed_T_j[0:3, 3]
            z_j = np.dot(fixed_R_j, joint_axes[j])
            o_j = fixed_t_j
            Jvi = np.cross(z_j, np.subtract(o_n, o_j))
            Jwi = z_j
            J[:, j] = np.concatenate([Jvi, Jwi])
        # print('world_frame any joint', J)
        return J