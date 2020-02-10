import rospy
from geometry_msgs.msg import Twist, TwistStamped
from teleop.command_publishers import CommandPublisherBaseClass


class TwistStampedCommandPublisher(CommandPublisherBaseClass):
    def __init__(self, config_data, section_name):
        super(TwistStampedCommandPublisher, self).__init__(config_data, section_name)
        self.command_type = 'twist_stamped'
        self.pub = rospy.Publisher(self.rostopic_publisher_cmd, TwistStamped, queue_size=1)

    def get_zero_command(self):
        zero_twist_stamped = TwistStamped()
        zero_twist_stamped.twist = Twist()
        zero_twist_stamped.twist.linear.x = 0.0
        zero_twist_stamped.twist.linear.y = 0.0
        zero_twist_stamped.twist.linear.z = 0.0
        zero_twist_stamped.twist.angular.x = 0.0
        zero_twist_stamped.twist.angular.y = 0.0
        zero_twist_stamped.twist.angular.z = 0.0
        return zero_twist_stamped

