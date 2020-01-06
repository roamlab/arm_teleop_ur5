import rospy
from std_msgs.msg import Int32, String
from teleop.utils.factory import make


class EndEffector:
    def __init__(self, config_data, section_name):
        # read the config file
        rostopic_action_set = config_data.get(section_name, 'rostopic_action_set')
        self.pub_action_set = rospy.Publisher(rostopic_action_set, Int32, queue_size=1)
        # hand specific
        command_publisher_section_name = config_data.get(section_name, 'command_publisher')
        self.command_publisher = make(config_data, command_publisher_section_name)
        self.command_type = config_data.get(section_name, 'command_type')

    def send_command(self, action):
        pass



