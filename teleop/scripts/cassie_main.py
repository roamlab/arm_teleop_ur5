#!/usr/bin/env python

#For sawyer and ur5
import math
import numpy
import time
from threading import Thread, Lock
import rospy
import tf
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from urdf_parser_py.urdf import URDF

import moveit_commander
import sys

from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive

from moveit_msgs.srv import GetStateValidity
from moveit_msgs.srv import GetStateValidityRequest
from moveit_msgs.srv import GetStateValidityResponse
from copy import deepcopy

from arm_teleoperation import *

#For sawyer robot
from moveit_msgs.msg import RobotState
#from intera_core_msgs.msg import JointCommand, EndpointState
#import intera_interface


# from intera_motion_interface import (
#     MotionTrajectory,
#     MotionWaypoint,
#     MotionWaypointOptions
# )
# from intera_motion_msgs.msg import TrajectoryOptions
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from teleop import Manipulator
from teleop import UserInterfaceDevice
from teleop import EndEffector
from teleop import CollisionChecker
from teleop.utils import read_config_file
from teleop import CartesianControl
from teleop import Kinematics
from teleop.utils import convert_pose_to_transform


class UR5Control(object):
    # Initialization
    def __init__(self, hand_is_attached):
        # Initialize moveit commander
        moveit_commander.roscpp_initialize(sys.argv)  # ## First initialize moveit_commander and rospy.
        self.moveit_robot = moveit_commander.RobotCommander()  ## Instantiate a RobotCommander object.
        self.scene = moveit_commander.PlanningSceneInterface()  # ## Instantiate a PlanningSceneInterface object.
        self.group = moveit_commander.MoveGroupCommander("manipulator")  ## Instantiate a MoveGroupCommander object.
        self.robot = URDF.from_parameter_server()  # Loads the robot model, which contains the robot's kinematics information

        # Initialize other classes needed for control
        self.cc = CartesianControl()
        self.collision_checker = CollisionChecker(self.group, hand_is_attached)
        self.kinematics = Kinematics(self.robot)

        # Subscribes to information about what the current joint values are.
        rospy.Subscriber("/joint_states", JointState, self.joint_callback)

        # Subscribes to command for end-effector pose
        rospy.Subscriber("/ur5_target_pose", Pose, self.command_callback)

        # # Publishes desired joint velocities
        self.pub_vel = rospy.Publisher("/JointVelocityMove", JointState, queue_size=1)
        # Publishes desired joint velocities
        # self.pub_vel = rospy.Publisher("/ur_driver/joint_speed", JointTrajectory, queue_size=1)

        # Initialize variables needed for publishing
        self.x_current = tf.transformations.identity_matrix()
        self.x_target = tf.transformations.identity_matrix()
        self.last_command_time = 0
        self.joint_velocity_msg = JointState()
        self.joint_velocity_msg.name = self.moveit_robot.get_current_state().joint_state.name

        # self.joint_velocity_msg = JointTrajectory()
        # self.joint_velocity_msg.joint_names = self.moveit_robot.get_current_state().joint_state.name
        # self.joint_velocity_msg.points = []
        # self.joint_velocity_point = JointTrajectoryPoint()
        # self.joint_velocity_msg.points.append(self.joint_velocity_point)

        # Initialize timer that will trigger callbacks
        self.mutex = Lock()
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)


    # def timer_callback(self, event):
    #     self.mutex.acquire()
    #     if time.time() - self.last_command_time < 0.5:
    #         dq = self.cc.cartesian_control(self.joint_transforms,
    #                                self.x_current, self.x_target)
    #         self.joint_velocity_msg.points[0].velocities = dq
    #         if not self.check_for_valid_position(self.moveit_robot.get_current_state(), dq):
    #             self.joint_velocity_msg.points[0].velocities = numpy.zeros(7)
    #         self.pub_vel.publish(self.joint_velocity_msg)
    #     else:
    #         self.joint_velocity_msg.points[0].velocities = numpy.zeros(7)
    #     self.mutex.release()

    def timer_callback(self, event):
        self.mutex.acquire()
        if time.time() - self.last_command_time < 0.5:
            start_cart = time.time()
            dq = self.cc.cartesian_control(self.joint_transforms,
                                           self.x_current, self.x_target)
            # print('end_cart', time.time() - start_cart)
            self.joint_velocity_msg.velocity = dq
            # print("new dq")
            if not self.check_for_valid_position(self.moveit_robot.get_current_state(), dq):
                self.joint_velocity_msg.velocity = numpy.zeros(6)
            self.pub_vel.publish(self.joint_velocity_msg)
        else:
            self.joint_velocity_msg.velocity = numpy.zeros(6)

        # print('published velocity command')
        self.mutex.release()

    def command_callback(self, command):
        self.mutex.acquire()
        self.target_pose = command
        self.x_target = convert_pose_to_transform(command)
        self.last_command_time = time.time()
        self.mutex.release()

    def joint_callback(self, joint_values):
        self.mutex.acquire()
        self.x_current = convert_pose_to_transform(self.group.get_current_pose().pose)

        root = self.robot.get_root()
        T = tf.transformations.identity_matrix()
        self.joint_transforms = self.kinematics.find_ur5_joint_transforms(root, T, joint_values)
        self.mutex.release()

    def check_for_valid_position(self, robot_state, robot_velocities):
        robot_state.joint_state.position = robot_state.joint_state.position + robot_velocities / 2  # Find robot position 0.5 seconds in the future
        # robot_state.attached_collision_objects.append(self.collision_checker.attached_trigrip)  #Attach trigrip for collision checking purposes
        return self.collision_checker.check_for_collisions(robot_state, "manipulator")



if __name__ == '__main__':
    rospy.init_node('cartesian_control', anonymous=True)
    robot = rospy.get_param('~robot', None)
    hand_attached = rospy.get_param('~hand_attached', False)  # Is there a hand on the sawyer
    hand_urdf_location = rospy.get_param('~hand_urdf',
                                         None)  # File location of urdf package "schunk_description"  - ..../schunk_description/urdf/sdh/schunk_end_effector.urdf.xacro

    cc = UR5Control(False)
    rospy.spin()
else:
    print
    "Unknown robot. Please input robot argument (either 'sawyer', 'sawyer_compliant' or 'ur5') in the command line"
