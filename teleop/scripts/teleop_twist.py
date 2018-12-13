#!/usr/bin/env python
# Teleopeartion node for a manipulator with an end-effector
# "twist" means that the command is given in linear and angular velocities format

import rospy
import PyKDL
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist, TwistStamped
import subprocess as sp
import os
import ConfigParser
import ast
# remove the following debugging util after finishing
import ipdb

def read_config_file(config_file_name):
    config_data = ConfigParser.ConfigParser()
    path = os.path.join(os.environ['ARM_TELEOP_UR5_SRC'],'teleop/config/',config_file_name)
    config_data.read(path)
    return config_data

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
        self.manipulator_comd = Twist()
        self.sub_manipulator_comd = rospy.Subscriber(
            rostopic_manipulator_comd, Twist, self.manipulator_comd_CB)
        # subscriber - end-effector command from user input 
        rostopic_manipulator_comd = config_data.get(
            'user_interface', 'rostopic_end_effector_comd')
        self.end_effector_comd = Int32()
        self.sub_end_effector_comd = rospy.Subscriber(
            rostopic_manipulator_comd, Int32, self.end_effector_comd_CB)

    def manipulator_comd_CB(self,msg_data):
        self.manipulator_comd = msg_data

    def end_effector_comd_CB(self,msg_data):
        self.end_effector_comd = msg_data

# class Manipulator:
#     def __init__(self):

class Teleop:
    def __init__(self, config_file_name):
        # read the config file
        config_data = read_config_file(config_file_name)
        self.config_file_name = config_file_name
        # init the teleop node
        rospy.init_node(config_data.get('teleop','name'))
        # 
        self.user_interface = UserInterfaceDevice(config_file_name)
        # config_data.read(path)
        # self.pub_comd = rospy.Publisher(hand_name+'/goal_action', Int32, queue_size=1)
        # self.pub_open_pose_def = rospy.Publisher(hand_name+'/open_pose_def', PoseLevels, queue_size=1)
        # self.pub_close_pose_def = rospy.Publisher(hand_name+'/close_pose_def', PoseLevels, queue_size=1)
        # self.sub_curr_rob_state = rospy.Subscriber(hand_name+"/current_state", RobState, self.rob_curr_state_CB)
        self.rate = rospy.Rate(config_data.getfloat(
            'teleop','rate_Hz'))
        # self.curr_rob_state=RobState

    def run(self):
    #   command_action = 'disabled'
        while not rospy.is_shutdown():
            sp.call('clear',shell=True)
            self.rate.sleep()

    # def rob_curr_state_CB(self, msg_data):
    #     self.curr_rob_state = msg_data
if __name__ == '__main__':
    try:
        teleop = Teleop('keyboard.cfg')
        teleop.run()

    except rospy.ROSInterruptException:
        pass

