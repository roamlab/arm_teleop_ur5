import rospy
import ast
import numpy as np
from sensor_msgs.msg import JointState
from teleop.command_publishers import CommandPublisherBaseClass


class JointStateCommandPublisher(CommandPublisherBaseClass):
    def __init__(self, config_data, section_name):
        super(JointStateCommandPublisher, self).__init__(config_data, section_name)
        self.command_type = 'joint_state'
        self.num_joints = config_data.getint(section_name, 'num_joints')
        if config_data.has_option(section_name, 'joint_names'):
            self.joint_names = ast.literal_eval(config_data.get(section_name, 'joint_names'))
        self.pub = rospy.Publisher(self.rostopic_publisher_cmd, JointState, queue_size=1)

    def get_zero_command(self):
        zero_jointstate = JointState()
        zero_jointstate.position = np.zeros(self.num_joints, dtype=np.float)
        zero_jointstate.velocity = np.zeros(self.num_joints, dtype=np.float64)
        return zero_jointstate

