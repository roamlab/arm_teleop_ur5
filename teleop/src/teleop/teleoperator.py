import rospy
import time
from threading import Lock
from geometry_msgs.msg import Pose, PoseStamped, Twist, TwistStamped
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
import copy
import numpy as np
import sys
from teleop import Manipulator
from teleop import CollisionChecker
from teleop.utils.transform_helpers import convert_pose_to_transform
from teleop.utils.transform_helpers import convert_twist_to_numpy_array
from teleop.utils.transform_helpers import convert_numpy_array_to_twist
from teleop import EndEffector
from teleop.utils.factory import make
import moveit_commander


class Teleoperator:
    def __init__(self, config_data, section_name):
        # read the config file
        # init the teleop node
        rospy.init_node(config_data.get(section_name, 'topic_name'))
        self.rate = rospy.Rate(config_data.getfloat(section_name, 'rate_hz'))
        self.mutex = Lock()

        self._init_manipulator(config_data, section_name)
        self._init_end_effector(config_data, section_name)
        self._init_collision_checker(config_data, section_name)

        # init cartesian controller
        cartesian_controller_section_name = config_data.get(section_name, "cartesian_controller")
        self.cart_ctrl = make(config_data, cartesian_controller_section_name)

        # self.velocity_scale = ast.literal_eval(config_data.get(section_name, 'velocity_scale'))
        self.zero_needed = False

    # TODO: check frames
    # def check_frames(self):
    #     assert self.manipulator.reference_frame == self.manipulator_user_interface.reference_frame
    #     assert self.manipulator.moving_frame == self.manipulator_user_interface.moving_frame
    #     assert self.manipulator.command_publisher.reference_frame == self.manipulator.

    def _init_end_effector(self, config_data, section_name):
        # initialize end effector if one is defined in config
        self.end_effector_user_interface = None
        self.end_effector = None
        if config_data.has_option(section_name, 'end_effector_user_interface_device'):
            end_effector_user_interface_device_section_name = config_data.get(section_name,
                                                                              'end_effector_user_interface_device')
            self.end_effector_user_interface = make(config_data, end_effector_user_interface_device_section_name)
            self.end_effector = EndEffector(config_data, section_name)

    def _init_manipulator(self, config_data, section_name):
        # initialize manipulator if one is defined in config
        self.manipulator_user_interface = None
        self.manipulator = None
        if config_data.has_option(section_name, 'manipulator_user_interface_device'):
            manipulator_user_interafce_device_name = config_data.get(section_name, 'manipulator_user_interface_device')
            self.manipulator_user_interface = make(config_data, manipulator_user_interafce_device_name)
            manipulator_section_name = config_data.get(section_name, 'manipulator')
            self.manipulator = Manipulator(config_data, manipulator_section_name)

    def _init_collision_checker(self, config_data, section_name):
        # init collision checker if one is defined in config
        self.check_for_collision = config_data.getboolean(section_name, 'check_for_collision')
        if self.check_for_collision:
            moveit_commander.roscpp_initialize(sys.argv)  # First initialize moveit_commander and rospy.
            self.collision_checker = CollisionChecker()
            self.collision_horizon = config_data.getfloat(section_name, 'collision_horizon')
            self.collision_dt = config_data.getfloat(section_name, 'collision_dt')

    def compute_manipulator_command(self):
        """
        self.manipulator.pose_current is of type Pose()
        self.manipulator.joint_current is of type JointState()


        self.manipulator_user_interface.cmd is of type Pose() or Twist()
        Returns:

        """
        self.mutex.acquire()
        command = None
        if not self.manipulator_user_interface.cmd == []:
            if (time.time() - self.manipulator_user_interface.cmd_time) < .5:
                # if no command has been issued in .1 seconds, send a 0 command, otherwise, use regular command
                current_pose = copy.deepcopy(self.manipulator.pose_current)  # Pose() of the end effector in fixed frame
                fixed_T_ee_current = convert_pose_to_transform(current_pose)  # fixed frame to end effector transform
                transforms = copy.deepcopy(self.manipulator.transforms)

                # compute v_ee
                if isinstance(self.manipulator_user_interface.cmd, Twist):
                    command_twist = copy.deepcopy(self.manipulator_user_interface.cmd)  # Twist()
                    # transform twist into numpy array
                    v_ee = convert_twist_to_numpy_array(command_twist)
                elif isinstance(self.manipulator_user_interface.cmd, Pose):
                    desired_pose = copy.deepcopy(self.manipulator_user_interface.cmd)  # Pose()
                    v_ee = self.cart_ctrl.compute_vee(current_pose, desired_pose)
                else:
                    raise TypeError('user_interface.command_type is not recognized')
                if not np.allclose(np.zeros(6), v_ee):
                    # if v_ee is close to zero, command is None
                    qd = self.cart_ctrl.compute_qd(v_ee, transforms, fixed_T_ee_current, self.manipulator.joint_axes)
                    if self.check_for_collision and self.check_collision(qd, self.manipulator.joint_current):
                        print('collision detected, changing command to 0')
                    else:
                        if self.manipulator.command_type == 'twist_stamped':
                            command = TwistStamped()
                            command.twist = convert_numpy_array_to_twist(v_ee)
                        elif self.manipulator.command_type == 'joint_state':
                            command = JointState()
                            command.header.stamp = rospy.Time.now()
                            command.name = self.manipulator.joint_names
                            command.velocity = qd
                        else:
                            raise ValueError("self.manipulator.command_type:"
                                             " {} not recognized".format(self.manipulator.command_type))

        if command is None:
            if self.zero_needed:
                print('sending zero')
                command = self.manipulator.get_zero_command()
                self.manipulator.send_command(command)
                self.zero_needed = False
            else:
                print('Zero already sent')
        else:
            self.manipulator.send_command(command)
            self.zero_needed = True

        self.mutex.release()

    def compute_end_effector_command(self):
        if not self.end_effector_user_interface.cmd == []:
            self.end_effector.send_command(self.end_effector_user_interface.cmd)
        else:
            print('no commands issued')

    def check_collision(self, qd, q_current):
        """
        Args:
            qd (np.array): The computed joint velocities that are testing for future collisions
            q_current (JointState): The current joint values of the robot

        Returns:
            bool: A value that is True if collision is detected, and False if there is no collision

        """
        has_collision = False
        t = self.collision_dt
        while t <= self.collision_horizon:
            robot_state = RobotState()
            robot_state.joint_state = copy.deepcopy(q_current)
            future_position = (np.asarray(q_current.position) + qd*t).tolist()
            robot_state.joint_state.position = future_position
            is_valid = self.collision_checker.is_state_valid(robot_state, self.manipulator.group)
            has_collision = has_collision or not is_valid
            t = t+self.collision_dt
        return has_collision

    def run(self):
        while not rospy.is_shutdown():
            if self.manipulator:
                self.compute_manipulator_command()
            if self.end_effector:
                self.compute_end_effector_command()
            self.rate.sleep()
