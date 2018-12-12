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

class UserInterfaceDevice:
    def __init__(self, config_file_name):
        # parse the config file
        config_data = ConfigParser.ConfigParser()
        path = os.path.join(os.environ['ARM_TELEOP_UR5_SRC'],'teleop/config/',config_file_name)
        config_data.read(path)
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
        # command type and rostopics
        rostopic_manipulator_comd = config_data.get(
            'user_interface', 'rostopic_manipulator_comd')
        self.sub_manipulator_comd = rospy.Subscriber(
            rostopic_manipulator_comd, Twist, self.manipulator_comd_CB)
        # rostopic_end_effector_comd = config_data.get(
        #     'user_interface', 'rostopic_end_effector_comd')
        # self.pub_open_pose_def = rospy.Publisher(hand_name+'/open_pose_def', PoseLevels, queue_size=1)
        # self.pub_close_pose_def = rospy.Publisher(hand_name+'/close_pose_def', PoseLevels, queue_size=1)
        # self.sub_curr_rob_state = rospy.Subscriber(hand_name+"/current_state", RobState, self.rob_curr_state_CB)
        ipdb.set_trace()
    def manipulator_comd_CB(self,msg_data):
        a=0


# class Manipulator:
#     def __init__(self):

class Teleop:
    def __init__(self, config_file_name):
        # read the configuration file
        self.config_file_name = config_file_name
        self.user_interface = UserInterfaceDevice(config_file_name)
        # config_data.read(path)
        # rospy.init_node(hand_name+'_interface')
        # self.pub_comd = rospy.Publisher(hand_name+'/goal_action', Int32, queue_size=1)
        # self.pub_open_pose_def = rospy.Publisher(hand_name+'/open_pose_def', PoseLevels, queue_size=1)
        # self.pub_close_pose_def = rospy.Publisher(hand_name+'/close_pose_def', PoseLevels, queue_size=1)
        # self.sub_curr_rob_state = rospy.Subscriber(hand_name+"/current_state", RobState, self.rob_curr_state_CB)
        # self.rate = rospy.Rate(100) # 10hz
        # self.curr_rob_state=RobState

    def set_teleop_configuration(self):
        ipdb.set_trace()


    def run(self):
        a=self.config.get('user_interface','joint_angles_initial')
        ipdb.set_trace()
    #   command_action = 'disabled'
    #     while not rospy.is_shutdown():
    #         sp.call('clear',shell=True)
    #         self.rate.sleep()
        # joint_vel = np.array(ast.literal_eval(
        #     config_data.get('robot', 'joint_velocities_initial')))


    # def rob_curr_state_CB(self, msg_data):
    #     self.curr_rob_state = msg_data
if __name__ == '__main__':
    try:
        teleop = Teleop('keyboard.cfg')
        teleop.set_teleop_configuration()
        teleop.run()

    except rospy.ROSInterruptException:
        pass

