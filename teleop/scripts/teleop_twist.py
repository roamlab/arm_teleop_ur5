#!/usr/bin/env python
# Teleopeartion node for a manipulator with an end-effector
# "twist" means that the command is given in linear and angular velocities format

import rospy
import PyKDL
from geometry_msgs.msg import Pose, PoseStamped, Twist, TwistStamped
from tf_conversions import posemath as PoseMath
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
# from sensor_msgs.msg import JointState
import subprocess as sp
import os
import ConfigParser
import ast
import sys
# remove the following debugging util after finishing
# import ipdb

def read_config_file(config_file_name):
    config_data = ConfigParser.ConfigParser()
    path = os.path.join(os.environ['ARM_TELEOP_UR5_SRC'],'teleop/config/',config_file_name)
    config_data.read(path)
    return config_data

def convert_geometry_msgs_to_PyKDL_twist(msg_data):
    vel = msg_data.linear; rot = msg_data.angular;
    twist_PyKDL = PyKDL.Twist(
        PyKDL.Vector(vel.x,vel.y,vel.z), PyKDL.Vector(rot.x,rot.y,rot.z))
    return twist_PyKDL

def convert_PyKDL_to_geometry_msgs_twist(twist_PyKDL):
    twist = Twist()
    twist.linear.x = twist_PyKDL.vel.x()
    twist.linear.y = twist_PyKDL.vel.y()
    twist.linear.z = twist_PyKDL.vel.z()
    twist.angular.x = twist_PyKDL.rot.x()
    twist.angular.y = twist_PyKDL.rot.y()
    twist.angular.z = twist_PyKDL.rot.z()
    return twist



class UserInterfaceDevice:
    def __init__(self, config_file_name):
        # read the config file
        config_data = read_config_file(config_file_name)
        # device name
        self.name = config_data.get(
            'user_interface', 'name')
        # device frame offsets
        rot_X = ast.literal_eval(config_data.get(
            'user_interface', 'TF_device2view_rot_X'))
        rot_Y = ast.literal_eval(config_data.get(
            'user_interface', 'TF_device2view_rot_Y'))
        rot_Z = ast.literal_eval(config_data.get(
            'user_interface', 'TF_device2view_rot_Z'))
        pos = ast.literal_eval(config_data.get(
            'user_interface', 'TF_device2view_trans'))
        frame = PyKDL.Frame.Identity()
        frame.M = PyKDL.Rotation(
            rot_X[0],rot_Y[0],rot_Z[0],
            rot_X[1],rot_Y[1],rot_Z[1],
            rot_X[2],rot_Y[2],rot_Z[2])
        frame.P = PyKDL.Vector(pos[0],pos[1],pos[2])
        self.tf_dev2view = frame
        # command_type
        self.command_type = config_data.get(
            'user_interface', 'command_type')
        # subscriber - manipulator command from user input
        rostopic_manipulator_comd = config_data.get(
            'user_interface', 'rostopic_manipulator_comd')
        self.manipulator_comd = PyKDL.Twist()
        self.sub_manipulator_comd = rospy.Subscriber(
            rostopic_manipulator_comd, Twist, self.manipulator_comd_CB)
        # subscriber - end-effector command from user input 
        rostopic_manipulator_comd = config_data.get(
            'user_interface', 'rostopic_end_effector_comd')
        self.end_effector_comd = 0
        self.sub_end_effector_comd = rospy.Subscriber(
            rostopic_manipulator_comd, Int32, self.end_effector_comd_CB)

    def manipulator_comd_CB(self,msg_data):
        # self.manipulator_comd is in in the type of [PrKDL.Twist]
        self.manipulator_comd = convert_geometry_msgs_to_PyKDL_twist(msg_data)

    def end_effector_comd_CB(self,msg_data):
        self.end_effector_comd = msg_data.data

class Manipulator:
    def __init__(self, config_file_name):
        # read the config file
        config_data = read_config_file(config_file_name)
        self.name = config_data.get(
            'manipulator', 'name')
        # manipulator frame offsets
        rot_X = ast.literal_eval(config_data.get(
            'manipulator', 'TF_view2robot_rot_X'))
        rot_Y = ast.literal_eval(config_data.get(
            'manipulator', 'TF_view2robot_rot_Y'))
        rot_Z = ast.literal_eval(config_data.get(
            'manipulator', 'TF_view2robot_rot_Z'))
        pos = ast.literal_eval(config_data.get(
            'manipulator', 'TF_view2robot_trans'))
        frame = PyKDL.Frame.Identity()
        frame.M = PyKDL.Rotation(
            rot_X[0],rot_Y[0],rot_Z[0],
            rot_X[1],rot_Y[1],rot_Z[1],
            rot_X[2],rot_Y[2],rot_Z[2])
        frame.P = PyKDL.Vector(pos[0],pos[1],pos[2])
        self.tf_view2robot = frame
        # command_type
        self.command_type = config_data.get(
            'manipulator', 'command_type')
        # command_reference_frame
        self.command_reference_frame = config_data.get(
            'manipulator', 'command_reference_frame')
        # subscriber - manipulator current pose
        rostopic_pose_current = config_data.get(
            'manipulator', 'rostopic_pose_current')
        self.pose_current = PyKDL.Frame.Identity()
        self.sub_pose_current = rospy.Subscriber(
            rostopic_pose_current, PoseStamped, self.pose_current_CB)
        # subscriber - manipulator current joint states
        rostopic_joint_current = config_data.get(
            'manipulator', 'rostopic_joint_current')
        self.joint_current = JointState()
        self.sub_joint_current = rospy.Subscriber(
            rostopic_joint_current, JointState, self.joint_current_CB)
        # publisher - manipulator commanded twist
        rostopic_twist_set = config_data.get(
            'manipulator', 'rostopic_twist_set')
        self.pub_twist_set = rospy.Publisher(
            rostopic_twist_set, TwistStamped, queue_size=1)
        # publisher - manipulator commanded pose
        rostopic_pose_set = config_data.get(
            'manipulator', 'rostopic_pose_set')
        self.pub_pose_set = rospy.Publisher(
            rostopic_pose_set, PoseStamped, queue_size=1)

    def pose_current_CB(self, msg_data):
        self.pose_current = PoseMath.fromMsg(msg_data.pose)

    def joint_current_CB(self, msg_data):
        self.joint_current = msg_data

class Teleop:
    def __init__(self, config_file_name):
        # read the config file
        config_data = read_config_file(config_file_name)
        self.config_file_name = config_file_name
        self.velocity_scale = ast.literal_eval(config_data.get(
            'teleop', 'velocity_scale'))
        # init the teleop node
        rospy.init_node(config_data.get('teleop','name'))
        self.rate = rospy.Rate(config_data.getfloat(
            'teleop','rate_Hz'))
        # init user interface
        self.user_interface = UserInterfaceDevice(config_file_name)
        # init manipulator
        self.manipulator = Manipulator(config_file_name)
        # teleopeartion master to slave transformation
        self.tf_master2robot = (
            self.manipulator.tf_view2robot * self.user_interface.tf_dev2view)

    def compute_send_command(self):
        # for now, only twist mode is supported
        compatible = (self.manipulator.command_type=="twist")
        if compatible:
            command_twist_in_robot = (
                self.tf_master2robot*self.user_interface.manipulator_comd)
            if (self.manipulator.command_reference_frame=="end_effector_frame"):
                command_twist_in_robot_base = (
                    self.manipulator.pose_current*command_twist_in_robot)
            else:
                command_twist_in_robot_base = command_twist_in_robot
            command_twist_scaled = PyKDL.Twist(
                command_twist_in_robot_base.vel*self.velocity_scale[0],
                command_twist_in_robot_base.rot*self.velocity_scale[1])
            command_twist_msg = TwistStamped()
            command_twist_msg.twist = (
                convert_PyKDL_to_geometry_msgs_twist(command_twist_scaled))
            self.manipulator.pub_twist_set.publish(command_twist_msg)
        else:
            print("For now, only twist mode is supported!") 

    def run(self):
        # check if command types from the two sides match
        match = (
            self.manipulator.command_type == self.user_interface.command_type)
        if not match:
            print("Command types of user interface and slave robot do not match!") 

        while not rospy.is_shutdown():
            # check if 
            if match:
                self.compute_send_command()
                sp.call('clear',shell=True)
            self.rate.sleep()

    # def rob_curr_state_CB(self, msg_data):
    #     self.curr_rob_state = msg_data
if __name__ == '__main__':
    try:
        teleop = Teleop(sys.argv[1])
        teleop.run()

    except rospy.ROSInterruptException:
        pass

