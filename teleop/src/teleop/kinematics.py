#!/usr/bin/env python
import math
import numpy
import tf
from teleop.utils import FrameAdapter


class Kinematics(object):
    def __init__(self, robot):
        self.robot = robot
        # self.robot = robot
        self.num_joints = 0
        self.joint_transforms = []
        self.joint_names = []
        self.joint_axes = []
        self.x_current = tf.transformations.identity_matrix()
        self.get_joint_info()
        self.reference_frame = self.robot.get_root()

    def get_joint_info(self):
        '''This is a function which will collect information about the robot which
           has been loaded from the parameter server. It will populate the variables
           self.num_joints (the number of joints), self.joint_names and
           self.joint_axes (the axes around which the joints rotate)'''
        link = self.robot.get_root()
        while True:
            if link not in self.robot.child_map:
                break
            (joint_name, next_link) = self.robot.child_map[link][0]
            current_joint = self.robot.joint_map[joint_name]
            if current_joint.type != 'fixed':
                self.num_joints = self.num_joints + 1
                self.joint_names.append(current_joint.name)
                self.joint_axes.append(current_joint.axis)
            link = next_link

    def align_with_z(self, axis):
        T = tf.transformations.identity_matrix()
        z = numpy.array([0, 0, 1])
        x = numpy.array([1, 0, 0])
        dot = numpy.dot(z, axis)
        if dot == 1:
            return T
        if dot == -1:
            return tf.transformation.rotation_matrix(math.pi, x)
        rot_axis = numpy.cross(z, axis)
        angle = math.acos(dot)
        return tf.transformations.rotation_matrix(angle, rot_axis)

    def compute_joint_transforms(self, joint_values):
        """
        Computes transforms from world to each joint (j) -> fixed_T_j
        From Matei's Intro to Robotics Notes: Kinematics.pdf
        T_li = fixed transforms. Do not change at run-time. They reflect the physical dimensions of the robot links (L_i)
        T_ji(q_i) = variable transforms. They reflect the joint (J_i) values of the robot at any given time
        * note that J_i moves L_i

        fixed_T_tool = T_lb*(/PRODUCT_(i=0)^(n-1))[T_ji*T_li(qi)]
        fixed_T_tool = T_lb*[T_j0(q0)*T_l0]*[T_j1(q1)*T_l1]*[T_j2(q2)*T_l2]

        Args:
            joint_values: q values for each joint

        Returns:
            joint_transforms is list of fixed_T_j for each joint in robot
            T is fixed_T_tool

        """
        joint_transforms = []

        link = self.robot.get_root()

        T = tf.transformations.identity_matrix()
        while True:
            if link not in self.robot.child_map:
                break

            (joint_name, next_link) = self.robot.child_map[link][0]
            joint = self.robot.joint_map[joint_name]
            T_l = numpy.dot(tf.transformations.translation_matrix(joint.origin.xyz), tf.transformations.euler_matrix(joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2]))
            T = numpy.dot(T, T_l)

            if joint.type != "fixed":
                joint_transforms.append(T)
                q_index = self.joint_names.index(joint_name)
                T_j = tf.transformations.rotation_matrix(joint_values[q_index], numpy.asarray(joint.axis))
                T = numpy.dot(T, T_j)

            link = next_link

        return joint_transforms, T