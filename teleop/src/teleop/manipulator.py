import rospy
import ast
import numpy as np
from threading import Lock
from geometry_msgs.msg import Pose, PoseStamped, Twist, TwistStamped
from sensor_msgs.msg import JointState
from teleop import Kinematics
from teleop.utils.factory import make
from teleop.utils.transform_helpers import convert_transform_to_pose
from teleop.utils.frame_adapter import FrameAdapter
from urdf_parser_py.urdf import URDF


class Manipulator:
    def __init__(self, config_data, section_name):
        """
        Note, I am calling the end frame of the manipulator: tool (because end effector is going to get confusing)
        Args:
            config_data:
            section_name:
        """
        # read the config file
        self.mutex = Lock()
        self.kinematics = Kinematics(robot=URDF.from_parameter_server())
        command_publisher_section_name = config_data.get(section_name, 'command_publisher')
        self.command_publisher = make(config_data, command_publisher_section_name)
        # # command_publisher_reference_frame to kinematics_reference_frame
        # self.cmd_pub_ref_T_kin_ref = np.asarray(ast.literal_eval(config_data.get(section_name, 'cmd_pub_ref_T_kin_ref')))
        # # kinematics_moving_frame to command_publisher_moving_frame
        # self.kin_mf_T_cmd_pub_mf = np.asarray(ast.literal_eval(config_data.get(section_name, 'kin_mf_T_cmd_pub_mf')))

        self.joint_current = JointState()
        self.joint_current.position = [0.0]*self.kinematics.num_joints
        self.transforms, self.T = self.kinematics.compute_joint_transforms(self.joint_current.position)
        # self.kin_transforms, self.kin_ref_T_kin_mf = self.kinematics.compute_joint_transforms(self.joint_current.position)
        # self.cmd_transforms = [np.dot(self.cmd_pub_ref_T_kin_ref, kin_ref_T_j) for kin_ref_T_j in self.kin_transforms]
        # self.cmd_ref_T_cmd_mf = np.dot(np.dot(self.cmd_pub_ref_T_kin_ref, self.kin_ref_T_kin_mf), self.kin_mf_T_cmd_pub_mf)

        self.group = config_data.get(section_name, 'moveit_group_name')

        # subscriber - manipulator current pose
        rostopic_pose_current = config_data.get(section_name, 'rostopic_pose_current')
        rospy.Subscriber(rostopic_pose_current, PoseStamped, self.pose_current_CB)
        self.pose_current = Pose()

        # subscriber - manipulator current joint states
        rostopic_joint_current = config_data.get(section_name, 'rostopic_joint_current')
        rospy.Subscriber(rostopic_joint_current, JointState, self.joint_current_CB)
        self.joint_current = JointState()
        # publisher - manipulator commanded twist
        self.frame_adapter = None
        if config_data.has_option(section_name, 'frame_adapter'):
            frame_adapter_section_name = config_data.get(section_name, 'frame_adapter')
            self.frame_adapter = FrameAdapter(config_data, frame_adapter_section_name)

    def pose_current_CB(self, msg_data):
        """ Sets self.pose_current when something is published to rostopic_pose_current
            msg_data (PoseStamped):

        """
        self.mutex.acquire()
        self.pose_current = msg_data.pose
        self.mutex.release()

    def joint_current_CB(self, msg_data):
        """ Sets self.joint_current when something is published to rostopic_joint_current

        Args:
            msg_data (JointState):

        """
        self.mutex.acquire()
        self.joint_current = msg_data
        self.transforms, self.T = self.kinematics.compute_joint_transforms(self.joint_current.position)
        if self.frame_adapter is not None:
            self.transforms = [self.frame_adapter.change_fixed_frame(fixed_T_j) for fixed_T_j in self.transforms]
            self.T = self.frame_adapter.change_moving_frame(self.frame_adapter.change_fixed_frame(self.T))
        self.mutex.release()

    def send_command(self, command):
        self.mutex.acquire()
        self.command_publisher.publish(command)
        self.mutex.release()

    def get_zero_command(self):
        return self.command_publisher.get_zero_command()

    @property
    def command_type(self):
        return self.command_publisher.command_type

    @property
    def joint_axes(self):
        return self.kinematics.joint_axes

    @property
    def joint_names(self):
        return self.kinematics.joint_names

