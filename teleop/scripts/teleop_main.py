#!/usr/bin/env python
# Teleopeartion node for a manipulator with an end-effector
# "twist" means that the command is given in linear and angular velocities format

import rospy
import PyKDL
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, Twist, TwistStamped
from tf_conversions import posemath as PoseMath
from std_msgs.msg import Int32, String
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy
from nasa_hand_msgs.msg import PoseLevels as NasaHandPoseLevels
import subprocess as sp
import os
import ConfigParser
import ast
import sys
# remove the following debugging util after finishing
import ipdb

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

class EndEffectorCommand:
    def __init__(self, config_data):
        # read the config file
        self.action = 'disable'
        self.joy_buttons_def =  ast.literal_eval(config_data.get(
            'user_interface', 'joy_buttons_def'))

    def decode_from_joy(self,joy):
        self.action = "pause"
        for action in self.joy_buttons_def:
            if ((joy.buttons[self.joy_buttons_def.index(action)]==1) 
                and (action!="N/A")):
                self.action = action

class UserInterfaceDevice:
    def __init__(self, config_file_name):
        # read the config file
        config_data = read_config_file(config_file_name)
        # device name
        self.name = config_data.get(
            'user_interface', 'name')
        self.load_manipulator_comd_config(config_data)
        self.load_end_effector_comd_config(config_data)

    def load_manipulator_comd_config(self,config_data):
        # this func load configurations about the manipulator command that the user_interface gives
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
        if (self.command_type=="pose"):
            self.manipulator_comd = []
            rospy.Subscriber(
                rostopic_manipulator_comd, Pose, self.manipulator_comd_pose_CB)            
        else:
            self.manipulator_comd = PyKDL.Twist()
            rospy.Subscriber(
                rostopic_manipulator_comd, Twist, self.manipulator_comd_twist_CB)

    def load_end_effector_comd_config(self,config_data):
        # this func load configurations about the end-effector command that the user_interface gives
        # subscriber - end-effector command from user input 
        # it is assumed that the joy states are sent
        rostopic_end_effector_comd = config_data.get(
            'user_interface', 'rostopic_end_effector_comd')
        self.end_effector_comd = EndEffectorCommand(config_data)
        self.sub_end_effector_comd = rospy.Subscriber(
            rostopic_end_effector_comd, Joy, self.end_effector_comd_CB)

    def manipulator_comd_pose_CB(self,msg_data):
        # msg_data is received in the type of [geometry_msgs/Pose]
        self.manipulator_comd = PoseMath.fromMsg(msg_data)

    def manipulator_comd_twist_CB(self,msg_data):
        # msg_data is received in the type of [geometry_msgs/twist]
        # self.manipulator_comd is presumed in the type of [PrKDL.Twist]
        self.manipulator_comd = convert_geometry_msgs_to_PyKDL_twist(msg_data)

    def end_effector_comd_CB(self,msg_data):
        # msg_data is received in the type of [sensor_msgs/Joy]
        # this callback extracts the buttons that correspond to end-effector actions
        self.end_effector_comd.decode_from_joy(msg_data)

class Manipulator:
    def __init__(self, config_file_name):
        # read the config file
        config_data = read_config_file(config_file_name)
        self.name = config_data.get(
            'manipulator', 'name')
        self.load_comd_config(config_data)
        self.load_communication_config(config_data)

    def load_comd_config(self,config_data):
        # this func loads the command configuration of the manipulator
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
        if (self.command_type=="pose"):
            self.load_resolved_rates_config(config_data)
    
    def load_communication_config(self, config_data):
        # this func loads the communication configurations, e.g. ros topics for current pose and joint
        # subscriber - manipulator current pose
        rostopic_pose_current = config_data.get(
            'manipulator', 'rostopic_pose_current')
        self.pose_current = PyKDL.Frame.Identity()
        rospy.Subscriber(rostopic_pose_current, PoseStamped, self.pose_current_CB)
        # subscriber - manipulator current joint states
        rostopic_joint_current = config_data.get(
            'manipulator', 'rostopic_joint_current')
        self.joint_current = JointState()
        rospy.Subscriber(rostopic_joint_current, JointState, self.joint_current_CB)
        # publisher - manipulator commanded twist
        rostopic_twist_set = config_data.get(
            'manipulator', 'rostopic_twist_set')
        self.pub_twist_set = rospy.Publisher(
            rostopic_twist_set, TwistStamped, queue_size=1)

    def pose_current_CB(self, msg_data):
        self.pose_current = PoseMath.fromMsg(msg_data.pose)

    def joint_current_CB(self, msg_data):
        self.joint_current = msg_data

    def resolved_rates(self,desiredPose):
        # compute pose error (result in kdl.twist format)
        poseError = PyKDL.diff(self.pose_current,desiredPose)
        posErrNorm = poseError.vel.Norm()
        rotErrNorm = poseError.rot.Norm()
        # compute velocity magnitude based on position error norm
        if (posErrNorm > self.resolvedRatesConfig['tolPos']):
            tolPosition = self.resolvedRatesConfig['tolPos']
            lambdaVel = self.resolvedRatesConfig['velRatio']
            velMax = self.resolvedRatesConfig['velMax']
            velMin = self.resolvedRatesConfig['velMin']
            if posErrNorm > (lambdaVel * tolPosition):
                velMag = velMax
            else:
                velMag = (velMin \
                          + (posErrNorm - tolPosition) \
                          * (velMax-velMin) / (tolPosition * (lambdaVel-1)))
        else:
            velMag = 0.0
        # compute angular velocity magnitude based on rotation error norm
        if rotErrNorm > self.resolvedRatesConfig['tolRot']:
            tolRotation = self.resolvedRatesConfig['tolRot']
            lambdaRot = self.resolvedRatesConfig['rotRatio']
            angVelMax = self.resolvedRatesConfig['angVelMax']
            angVelMin = self.resolvedRatesConfig['angVelMin']
            if rotErrNorm > (lambdaRot * tolRotation):
                angVelMag = angVelMax
            else:
                angVelMag = (angVelMin \
                             + (rotErrNorm - tolRotation) \
                             * (angVelMax - angVelMin) \
                             / (tolRotation * (lambdaRot-1)))
        else:
            angVelMag = 0.0
        # The resolved rates is implemented as Nabil Simaan's notes
        # apply both the velocity and angular velocity in the error pose direction
        desiredTwist = PyKDL.Twist()
        poseError.vel.Normalize() # normalize to have the velocity direction
        desiredTwist.vel = poseError.vel * velMag
        poseError.rot.Normalize() # normalize to have the ang vel direction
        desiredTwist.rot = poseError.rot * angVelMag
        return desiredTwist

    def send_command_twist(self,command_twist):
        command_twist_msg = TwistStamped()
        command_twist_msg.twist = (
            convert_PyKDL_to_geometry_msgs_twist(command_twist))
        self.pub_twist_set.publish(command_twist_msg)

    def load_resolved_rates_config(self,config_data):
        velMin = config_data.getfloat('resolved_rates', 'velMin') # minimum linear velocity [mm/sec]
        velMax = config_data.getfloat('resolved_rates', 'velMax') # maximum linear velocity [mm/sec]
        angVelMin = config_data.getfloat('resolved_rates', 'angVelMin') # minimum angular velocity [deg/sec]
        angVelMax = config_data.getfloat('resolved_rates', 'angVelMax') # maximum angular velocity [deg/sec]
        tolPos = config_data.getfloat('resolved_rates', 'tolPos') # positional tolerance [mm]
        tolRot = config_data.getfloat('resolved_rates', 'tolRot') # rotational tolerance [degree]
        velRatio = config_data.getfloat('resolved_rates', 'velRatio')
        rotRatio = config_data.getfloat('resolved_rates', 'rotRatio')
        self.resolvedRatesConfig = \
        {   'velMin': velMin / 1000.0, # minimum linear velocity [m/sec]
            'velMax': velMax / 1000.0, # maximum linear velocity [m/sec]
            'angVelMin': angVelMin / 180.0 * np.pi, # minimum angular velocity [rad/sec]
            'angVelMax': angVelMax / 180.0 * np.pi, # maximum angular velocity [rad/sec]
            'tolPos': tolPos / 1000.0, # positional tolerance [m]
            'tolRot': tolRot / 180.0 * np.pi, # rotational tolerance [rad]
            'velRatio': velRatio, # the ratio of max velocity error radius to tolarance radius, this value >1
            'rotRatio': rotRatio  # the ratio of max angular velocity error radius to tolarance radius, this value >1
        }


class EndEffector:
    def __init__(self, config_file_name):
        # read the config file
        config_data = read_config_file(config_file_name)
        self.name = config_data.get(
            'end_effector', 'name')
        self.type = config_data.get(
            'end_effector', 'type')
        self.rate_Hz = config_data.getfloat(
            'teleop', 'rate_Hz')
        rostopic_action_set = config_data.get(
            'end_effector', 'rostopic_action_set')
        self.pub_action_set = rospy.Publisher(
            rostopic_action_set, Int32, queue_size=1)
        # hand specific
        self.hand_open_def = NasaHandPoseLevels()
        self.hand_open_def.flex=0.0
        self.hand_open_def.spread=1.0
        rostopic_hand_open_def = config_data.get(
            'end_effector', 'rostopic_hand_open_def')
        self.pub_hand_open_def = rospy.Publisher(
            rostopic_hand_open_def, NasaHandPoseLevels, queue_size=1)

    def send_command(self, action):
        if (action=="grasp"):
            self.pub_action_set.publish(1)
        elif (action=="open"):
            self.pub_action_set.publish(2)
        elif ((action=="spread") or (action=="unspread")):
            if (action=="spread"):
                spread = self.hand_open_def.spread + 10/self.rate_Hz
            else:
                spread = self.hand_open_def.spread - 10/self.rate_Hz
            if (spread>=1):
                spread = 0.999
            elif (spread<0.6):
                spread = 0.6
            self.hand_open_def.spread = spread
            if (spread<=1):
                self.pub_hand_open_def.publish(self.hand_open_def)
            self.pub_action_set.publish(2)
        elif (action=="pause"):
            self.pub_action_set.publish(6)
        elif (action=="disable"):
            self.pub_action_set.publish(0)
        else:
            print("Wrong input for end-effector action, "+str(action),"\n")

class Teleop:
    def __init__(self, config_file_name):
        # read the config file
        config_data = read_config_file(config_file_name)
        self.config_file_name = config_file_name
        # init the teleop node
        rospy.init_node(config_data.get('teleop','name'))
        self.rate = rospy.Rate(config_data.getfloat(
            'teleop','rate_Hz'))
        # init user interface
        self.user_interface = UserInterfaceDevice(config_file_name)
        # init manipulator
        self.manipulator = Manipulator(config_file_name)
        # end-effector
        self.end_effector = EndEffector(config_file_name)
        # if a teleop reload_config option is available in the cfg file
        if (config_data.has_option('teleop','rostopic_reload_config')):
            rostopic_reload_config = config_data.get('teleop','rostopic_reload_config')
            rospy.Subscriber(
                rostopic_reload_config, String, self.reload_teleop_config_CB)            
        # update teleop parameters 
        self.update_teleop_config(config_data)

    def reload_teleop_config_CB(self, string_msg):
        # we use this func to reload a configuration file
        # this is to change the teleoperation parameters on the fly.
        config_file_name = string_msg.data
        config_data = read_config_file(config_file_name)
        self.manipulator.load_comd_config(config_data)
        self.user_interface.load_manipulator_comd_config(config_data)
        print("Configuration file ["+config_file_name+"] reloaded")
        # update teleop parameters 
        self.update_teleop_config(config_data)

    def update_teleop_config(self, config_data):
        #  this func is used in two places:
        #   (1) init
        #   (2) reloading a config
        #   [Important] when updated transforms are reloaded, this func HAS TO BE CALLED
        #   This because "tf_master2robot" needs to be updated by using all updated transformations
                # teleopeartion master to slave transformation
        self.tf_master2robot = (
            self.manipulator.tf_view2robot * self.user_interface.tf_dev2view)
        self.velocity_scale = ast.literal_eval(config_data.get(
            'teleop', 'velocity_scale'))

    def compute_send_command(self):
        # send manipulator command
        # for now, only twist mode is supported
        compatible = (
            self.manipulator.command_type==self.user_interface.command_type)
        if (compatible and (self.manipulator.command_type=="twist")):
            command_twist_in_robot = (
                self.tf_master2robot*self.user_interface.manipulator_comd)
            if (self.manipulator.command_reference_frame=="end_effector_frame"):
                command_twist_in_robot_base = (
                    self.manipulator.pose_current*command_twist_in_robot)
            else:
                command_twist_in_robot_base = command_twist_in_robot
            command_twist_scaled = self.scale_twist_command(command_twist_in_robot_base)
            self.manipulator.send_command_twist(command_twist_scaled)
        elif (compatible and (self.manipulator.command_type=="pose")
                and (not (self.user_interface.manipulator_comd==[]))):
            command_pose_in_robot = (
                self.tf_master2robot*self.user_interface.manipulator_comd)
            if (self.manipulator.command_reference_frame=="end_effector_frame"):
                command_pose_in_robot_base = (
                    self.manipulator.pose_current*command_pose_in_robot)
            else:
                command_pose_in_robot_base = command_pose_in_robot
            command_twist = self.manipulator.resolved_rates(command_pose_in_robot_base)
            self.manipulator.send_command_twist(command_twist)
        elif (compatible and (self.manipulator.command_type=="pose")
                and (self.user_interface.manipulator_comd==[])):
            pass
        else:
            print("Command_type(s) of Manipulator and User_interface do not match") 
        # send end-effector command
        self.end_effector.send_command(self.user_interface.end_effector_comd.action)

    def scale_twist_command(self,command_twist):
        command_twist_scaled = PyKDL.Twist(
            command_twist.vel*self.velocity_scale[0],
            command_twist.rot*self.velocity_scale[1])
        return command_twist_scaled

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
                # sp.call('clear',shell=True)
            self.rate.sleep()

    # def rob_curr_state_CB(self, msg_data):
    #     self.curr_rob_state = msg_data
if __name__ == '__main__':
    try:
        teleop = Teleop(sys.argv[1])
        teleop.run()

    except rospy.ROSInterruptException:
        pass
