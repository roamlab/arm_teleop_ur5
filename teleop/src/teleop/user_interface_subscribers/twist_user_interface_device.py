import rospy
from geometry_msgs.msg import Twist, TwistStamped
from teleop.user_interface_subscribers import UserInterfaceSubscriberBaseClass


class TwistUserInterfaceSubscriber(UserInterfaceSubscriberBaseClass):
    def __init__(self, config_data, section_name):
        super(TwistUserInterfaceSubscriber, self).__init__(config_data, section_name)
        self.command_type = 'twist'
        rospy.Subscriber(self.rostopic_subscriber_cmd, Twist, self.cmd_CB)

