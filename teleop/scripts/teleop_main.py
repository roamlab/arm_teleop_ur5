#!/usr/bin/env python
# Teleopeartion node for a manipulator with an end-effector
# "twist" means that the command is given in linear and angular velocities format

import rospy
import sys
import ConfigParser
from teleop import Teleoperator
# remove the following debugging util after finishing
import ipdb


if __name__ == '__main__':
    try:
        config_file = sys.argv[1]
        config_data = ConfigParser.ConfigParser()
        config_data.read(config_file)
        teleop = Teleoperator(config_data, 'teleop')
        teleop.run()

    except rospy.ROSInterruptException:
        pass
