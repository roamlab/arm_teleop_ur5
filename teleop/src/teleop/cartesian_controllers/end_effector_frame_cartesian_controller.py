#!/usr/bin/env python
import math
import numpy as np
import tf
from teleop.cartesian_controllers import CartesianControllerBaseClass
from teleop.utils.transform_helpers import convert_pose_to_transform


class EndEffectorFrameCartesianController(CartesianControllerBaseClass):
    def __init__(self, config_data, section_name):
        super(EndEffectorFrameCartesianController, self).__init__(config_data, section_name)

    def compute_vee(self, current_pose, desired_pose):
        """

        Args:
            current_pose (geometry_msgs Pose): The current pose of the manipulator
            desired_pose (geometry_msgs Pose): The desired pose of the manipulator

        Returns:
            v_ee (np.array): The end effector velocity that brings the manipulator closer to the desired pose.
                This velocity is given in the moving frame

        """
        fixed_T_tool_current = convert_pose_to_transform(current_pose)
        fixed_T_tool_desired = convert_pose_to_transform(desired_pose)
        # compute transform from current end-effector pose to desired one
        tool_cur_T_tool_des = np.dot(tf.transformations.inverse_matrix(fixed_T_tool_current), fixed_T_tool_desired)
        # get desired translational velocity in local frame
        dx = np.zeros(3, dtype=np.float64)
        dx[0] = tool_cur_T_tool_des[0][3]
        dx[1] = tool_cur_T_tool_des[1][3]
        dx[2] = tool_cur_T_tool_des[2][3]

        # normalize
        # to obtain max end-effector velocity of 0.1m/s
        if np.linalg.norm(dx) > 0.3:
            dx = (0.1 / np.linalg.norm(dx)) * dx

        # get desired angular velocity in local frame

        tool_cur_R_tool_des = tool_cur_T_tool_des[0:3, 0:3]
        angle, axis = self.rotation_from_matrix(tool_cur_R_tool_des)
        dw = angle * axis
        # normalize to max end-effector angular velocity of 1 rad/s
        if np.linalg.norm(dw) > 1.0:
            dw = (1.0 / np.linalg.norm(dw)) * dw

        # assemble translational and angular velocities
        v_ee = np.zeros(6, dtype=np.float64)
        v_ee[0:3] = dx
        v_ee[3:6] = dw
        return v_ee

    def compute_qd(self, v_ee, joint_transforms, fixed_T_tool_current, joint_axes=None):
        """

        Args:
            v_ee (np.array): velocity of the end effector given in the current end effector frame
            joint_transforms (list of np.arrays): specifies the transforms from the fixed frame to each joint
            fixed_T_tool_current (np.array): the final transform from the fixed frame to the current end effector frame
            joint_axes (list of np.arrays):

        Returns:
            qd: np.array

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
        """

        Args:
            joint_transforms:
            fixed_T_tool_current:
            joint_axes:

        Returns:

        """

        J = np.zeros((6, len(joint_transforms)))
        # --------------------------------------------------------------------------
        # FILL IN YOUR PART OF THE CODE FOR ASSEMBLING THE CURRENT JACOBIAN HERE

        for i in range(0, len(joint_transforms)):
            # rigid body transfer of joint rotation to end-effector motion
            fixed_T_j = joint_transforms[i]
            j_T_tool = np.dot(tf.transformations.inverse_matrix(fixed_T_j), fixed_T_tool_current)
            tool_T_j = tf.transformations.inverse_matrix(j_T_tool)
            tool_R_j = tool_T_j[0:3, 0:3]
            j_t_tool = j_T_tool[0:3, 3]
            S = np.zeros((3, 3))
            S[0, 1] = -j_t_tool[2]
            S[0, 2] = j_t_tool[1]
            S[1, 0] = j_t_tool[2]
            S[1, 2] = -j_t_tool[0]
            S[2, 0] = -j_t_tool[1]
            S[2, 1] = j_t_tool[0]
            RS = np.dot(tool_R_j, S)
            # choose the right column to put into Jacobian
            J[0:3, i] = - np.dot(RS, joint_axes[i])
            J[3:6, i] = np.dot(tool_R_j, joint_axes[i])

        rotation = np.identity(6)
        rotation[0:3, 0:3] = fixed_T_tool_current[0:3, 0:3]
        rotation[3:6, 3:6] = fixed_T_tool_current[0:3, 0:3]
        # print('rotated_ee_J', np.dot(rotation, J))
        return J

    def rotation_from_matrix(self, matrix):
        R = np.array(matrix, dtype=np.float64, copy=False)
        R33 = R[:3, :3]
        # axis: unit eigenvector of R33 corresponding to eigenvalue of 1
        l, W = np.linalg.eig(R33.T)
        i = np.where(abs(np.real(l) - 1.0) < 1e-8)[0]
        if not len(i):
            raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
        axis = np.real(W[:, i[-1]]).squeeze()
        # point: unit eigenvector of R33 corresponding to eigenvalue of 1
        l, Q = np.linalg.eig(R)
        i = np.where(abs(np.real(l) - 1.0) < 1e-8)[0]
        if not len(i):
            raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
        # rotation angle depending on axis
        cosa = (np.trace(R33) - 1.0) / 2.0
        if abs(axis[2]) > 1e-8:
            sina = (R[1, 0] + (cosa-1.0)*axis[0]*axis[1]) / axis[2]
        elif abs(axis[1]) > 1e-8:
            sina = (R[0, 2] + (cosa-1.0)*axis[0]*axis[2]) / axis[1]
        else:
            sina = (R[2, 1] + (cosa-1.0)*axis[1]*axis[2]) / axis[0]
        angle = math.atan2(sina, cosa)
        return angle, axis

    def S_matrix(self, w):
        S = np.zeros((3, 3))
        S[0, 1] = -w[2]
        S[0, 2] = w[1]
        S[1, 0] = w[2]
        S[1, 2] = -w[0]
        S[2, 0] = -w[1]
        S[2, 1] = w[0]
        return S
