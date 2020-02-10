import rospy
from geometry_msgs.msg import Pose, PoseStamped
from teleop.user_interface_subscribers import UserInterfaceSubscriberBaseClass


class PoseUserInterfaceSubscriber(UserInterfaceSubscriberBaseClass):
    def __init__(self, config_data, section_name):
        super(PoseUserInterfaceSubscriber, self).__init__(config_data, section_name)
        self.command_type = 'pose'
        rospy.Subscriber(self.rostopic_subscriber_cmd, Pose, self.cmd_CB)

