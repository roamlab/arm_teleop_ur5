import numpy as np


class ResolvedRatesCalculator(object):
    def __init__(self, config_data, section_name):
        # minimum linear velocity [m/sec]
        self.vel_min = config_data.getfloat(section_name, 'vel_min')/1000.0
        # maximum linear velocity [m/sec]
        self.vel_max = config_data.getfloat(section_name, 'vel_max')/1000.0
        # minimum angular velocity [rad/sec]
        self.ang_vel_min = config_data.getfloat(section_name, 'ang_vel_min')/180.0*np.pi
        # maximum angular velocity [rad/sec]
        self.ang_vel_max = config_data.getfloat(section_name, 'ang_vel_max')/180.0*np.pi
        # positional tolerance [m]
        self.pos_err_tolerance = config_data.getfloat(section_name, 'pos_err_tolerance')/1000.0
        # rotational tolerance [rad]
        self.rot_err_tolerance = config_data.getfloat(section_name, 'rot_err_tolerance')/180.0*np.pi
        # the ratio of max velocity error radius to tolarance radius, this value >1
        self.vel_lambda = config_data.getfloat(section_name, 'vel_lambda')
        # the ratio of max angular velocity error radius to tolarance radius, this value >1
        self.rot_lambda = config_data.getfloat(section_name, 'rot_lambda')

    def resolve_rates(self, pose_error):
        """

        Args:
            pose_error (np.array): error between current pose and desired pose.
                pose_error[0:3] are linear velocity
                pose_error[3:6] are angular velocity

        Returns:
            vel_mag (float): appropriate magnitude for the linear velocity to reduce pose error
            ang_vel_mag (float): appropriate magnitude for the angular velocity to reduce pose error

        """
        # poseError_TF = desired_pose * current_pose.Inverse()
        # positionError = poseError_TF.p
        # rotationError = poseError_TF.M.GetRot()
        pos_err_norm = np.linalg.norm(pose_error[0:3])
        rot_err_norm = np.linalg.norm(pose_error[3:6])
        # if you were to use pykdl, you could use
        # pos_err_norm = pose_error.linear.Norm()
        # rot_err_norm = pose_error.angular.Norm()
        # compute velocity magnitude based on position error norm
        if pos_err_norm > self.pos_err_tolerance:
            if pos_err_norm > (self.vel_lambda*self.pos_err_tolerance):
                vel_mag = self.vel_max
            else:
                vel_mag = self.vel_min + (pos_err_norm - self.pos_err_tolerance) * (self.vel_max - self.vel_min) / (
                            self.pos_err_tolerance * (self.vel_lambda - 1))
        else:
            vel_mag = 0.0
        # compute angular velocity magnitude based on rotation error norm
        if rot_err_norm > self.pos_err_tolerance:

            if rot_err_norm > (self.rot_lambda * self.rot_err_tolerance):
                ang_vel_mag = self.ang_vel_max
            else:
                ang_vel_mag = self.ang_vel_min + (rot_err_norm - self.rot_err_tolerance) * (
                            self.ang_vel_max - self.ang_vel_min) / (
                                      self.rot_err_tolerance * (self.rot_lambda - 1))
        else:
            ang_vel_mag = 0.0
        # The resolved rates is implemented as Nabil Simaan's notes
        # apply both the velocity and angular velocity in the error pose direction
        return vel_mag, ang_vel_mag
