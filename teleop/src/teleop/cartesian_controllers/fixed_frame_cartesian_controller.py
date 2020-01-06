#!/usr/bin/env python
import numpy as np
import PyKDL
from tf_conversions import posemath as PoseMath
from teleop.cartesian_controllers import CartesianControllerBaseClass
from teleop.utils.transform_helpers import convert_PyKDL_twist_to_geometry_msgs_twist
from teleop.utils.transform_helpers import convert_twist_to_numpy_array


class FixedFrameCartesianController(CartesianControllerBaseClass):
    def __init__(self, config_data, section_name):
        super(FixedFrameCartesianController, self).__init__(config_data, section_name)
        vel_min = config_data.getfloat(section_name, 'vel_min')  # minimum linear velocity [mm/sec]
        vel_max = config_data.getfloat(section_name, 'vel_max')  # maximum linear velocity [mm/sec]
        ang_vel_min = config_data.getfloat(section_name, 'ang_vel_min')  # minimum angular velocity [deg/sec]
        ang_vel_max = config_data.getfloat(section_name, 'ang_vel_max')  # maximum angular velocity [deg/sec]
        pos_err_tolerance = config_data.getfloat(section_name, 'pos_err_tolerance')  # positional tolerance [mm]
        rot_err_tolerance = config_data.getfloat(section_name, 'rot_err_tolerance')  # rotational tolerance [degree]
        vel_lambda = config_data.getfloat(section_name, 'vel_lambda')
        rot_lambda = config_data.getfloat(section_name, 'rot_lambda')
        self.resolvedRatesConfig = \
        {   'vel_min': vel_min / 1000.0,  # minimum linear velocity [m/sec]
            'vel_max': vel_max / 1000.0,  # maximum linear velocity [m/sec]
            'ang_vel_min': ang_vel_min / 180.0 * np.pi,  # minimum angular velocity [rad/sec]
            'ang_vel_max': ang_vel_max / 180.0 * np.pi,  # maximum angular velocity [rad/sec]
            'pos_err_tolerance': pos_err_tolerance / 1000.0,  # positional tolerance [m]
            'rot_err_tolerance': rot_err_tolerance / 180.0 * np.pi,  # rotational tolerance [rad]
            'vel_lambda': vel_lambda,  # the ratio of max velocity error radius to tolarance radius, this value >1
            'rot_lambda': rot_lambda  # the ratio of max angular velocity error radius to tolarance radius, this value >1
            }

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
        pose_error = PyKDL.diff(current_pydkl_frame, desired_pydkl_frame)
        # poseError_TF = desired_pose * current_pose.Inverse()
        # positionError = poseError_TF.p
        # rotationError = poseError_TF.M.GetRot()
        pos_err_norm = pose_error.vel.Norm()
        rot_err_norm = pose_error.rot.Norm()
        # compute velocity magnitude based on position error norm
        if pos_err_norm > self.resolvedRatesConfig['pos_err_tolerance']:
            pos_err_tolerance = self.resolvedRatesConfig['pos_err_tolerance']
            vel_lambda = self.resolvedRatesConfig['vel_lambda']
            vel_max = self.resolvedRatesConfig['vel_max']
            vel_min = self.resolvedRatesConfig['vel_min']
            if pos_err_norm > (vel_lambda * pos_err_tolerance):
                vel_mag = vel_max
            else:
                vel_mag = vel_min+(pos_err_norm - pos_err_tolerance)*(vel_max-vel_min)/(pos_err_tolerance*(vel_lambda-1))
        else:
            vel_mag = 0.0
        # compute angular velocity magnitude based on rotation error norm
        if rot_err_norm > self.resolvedRatesConfig['pos_err_tolerance']:
            rot_err_tolerance = self.resolvedRatesConfig['rot_err_tolerance']
            rot_lambda = self.resolvedRatesConfig['rot_lambda']
            ang_vel_max = self.resolvedRatesConfig['ang_vel_max']
            ang_vel_min = self.resolvedRatesConfig['ang_vel_min']
            if rot_err_norm > (rot_lambda * rot_err_tolerance):
                ang_vel_mag = ang_vel_max
                # print('ang_mag is max')
            else:
                ang_vel_mag = ang_vel_min+(rot_err_norm - rot_err_tolerance)*(ang_vel_max - ang_vel_min)/(rot_err_tolerance*(rot_lambda-1))
                # print('ang_mag is inbetween')
        else:
            # print('ang_mag is 0')
            ang_vel_mag = 0.0
        # The resolved rates is implemented as Nabil Simaan's notes
        # apply both the velocity and angular velocity in the error pose direction

        desiredTwist = PyKDL.Twist()
        # desiredTwist.vel = pose_error.vel
        # desiredTwist.rot = pose_error.rot
        # print('desiredTwist.vel', desiredTwist.vel)
        # print('desiredTwist.rot', desiredTwist.rot)
        # # normalize to obtain max end-effector velocity of 0.1m/s
        # if pose_error.vel.Norm() > 0.3:
        #     desiredTwist.vel = (0.1 / pose_error.vel.Norm()) * pose_error.vel
        #     print('desiredTwist.vel', desiredTwist.vel)
        # # normalize to max end-effector angular velocity of 1 rad/s
        # if pose_error.rot.Norm() > 1.0:
        #     desiredTwist.rot = (1.0 / pose_error.rot.Norm()) * pose_error.rot
        #     print('desiredTwist.rot', desiredTwist.rot)
        pose_error.vel.Normalize()  # normalize to have the velocity direction
        desiredTwist.vel = pose_error.vel * vel_mag
        pose_error.rot.Normalize()  # normalize to have the ang vel direction
        desiredTwist.rot = pose_error.rot * ang_vel_mag
        geo_twist = convert_PyKDL_twist_to_geometry_msgs_twist(desiredTwist)
        numpy_twist = convert_twist_to_numpy_array(geo_twist)
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