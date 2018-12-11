#!/usr/bin/env python
# Teleopeartion node for a robot arm

import rospy
import PyKDL
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist, TWistStamped
import subprocess as sp

class Teleop:
    def __init__(self, config_file_name):
        # read the configuration file
        rospy.init_node(hand_name+'_interface')
        self.pub_comd = rospy.Publisher(hand_name+'/goal_action', Int32, queue_size=1)
        self.pub_open_pose_def = rospy.Publisher(hand_name+'/open_pose_def', PoseLevels, queue_size=1)
        self.pub_close_pose_def = rospy.Publisher(hand_name+'/close_pose_def', PoseLevels, queue_size=1)
        self.sub_curr_rob_state = rospy.Subscriber(hand_name+"/current_state", RobState, self.rob_curr_state_CB)
        self.rate = rospy.Rate(100) # 10hz
        self.curr_rob_state=RobState

    def run(self):
        command_action = 'disabled';
        while not rospy.is_shutdown():
            sp.call('clear',shell=True)

            self.rate.sleep()


    def rob_curr_state_CB(self, msg_data):
        self.curr_rob_state = msg_data
    
if __name__ == '__main__':
    try:
        Nasa_Hand = UserInterface('nasa_hand')
        Nasa_Hand.run()
    except rospy.ROSInterruptException:
        pass
