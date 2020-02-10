#!/usr/bin/env python
from threading import Lock
import sys
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import InteractiveMarkerControl
import ConfigParser

class MarkerControl(object):

    #Initialization
    def __init__(self, config_data, section_name):
        self.mutex = Lock()        

        #Subscribes to information about what the current joint values are.
        rospy.Subscriber("/position_cartesian_current", PoseStamped, self.pose_callback)


        publisher_topic = config_data.get(section_name, 'publisher_topic')
        # Publishes Cartesian goals
        self.pub_command = rospy.Publisher(publisher_topic, Pose, queue_size=1)

        #This is where we hold the most recent joint transforms
        self.x_current = Pose()

        #Create "Interactive Marker" that we can manipulate in RViz
        self.cc_mode = True
        self.init_marker()
        self.ee_tracking = 0

        self.x_target = Pose()

        self.rate = 1.0/config_data.getfloat(section_name, 'rate_hz')

        self.timer = rospy.Timer(rospy.Duration(self.rate), self.timer_callback)

    def init_marker(self):

        self.server = InteractiveMarkerServer("control_markers")

        control_marker = InteractiveMarker()
        # control_marker.header.frame_id = self.robot.get_root()
        control_marker.header.frame_id = "base"
        control_marker.name = "cc_marker"

        move_control = InteractiveMarkerControl()
        move_control.name = "move_x"
        move_control.orientation.w = 1
        move_control.orientation.x = 1
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control_marker.controls.append(move_control)
        move_control = InteractiveMarkerControl()
        move_control.name = "move_y"
        move_control.orientation.w = 1
        move_control.orientation.y = 1
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control_marker.controls.append(move_control)
        move_control = InteractiveMarkerControl()
        move_control.name = "move_z"
        move_control.orientation.w = 1
        move_control.orientation.z = 1
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control_marker.controls.append(move_control)

        move_control = InteractiveMarkerControl()
        move_control.name = "rotate_x"
        move_control.orientation.w = 1
        move_control.orientation.x = 1
        move_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control_marker.controls.append(move_control)
        move_control = InteractiveMarkerControl()
        move_control.name = "rotate_y"
        move_control.orientation.w = 1
        move_control.orientation.z = 1
        move_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control_marker.controls.append(move_control)
        move_control = InteractiveMarkerControl()
        move_control.name = "rotate_z"
        move_control.orientation.w = 1
        move_control.orientation.y = 1
        move_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control_marker.controls.append(move_control)

        control_marker.scale = 0.75
        self.server.insert(control_marker, self.control_marker_feedback)

        # 'commit' changes and send to all clients
        self.server.applyChanges()

    def pose_callback(self, poseStamped):
        self.mutex.acquire()
        self.x_current = poseStamped.pose
        self.update_marker()
        self.mutex.release()

    def update_marker(self):
        self.server.setPose("cc_marker", self.x_current)
        self.server.applyChanges()

    def control_marker_feedback(self, feedback):
        if feedback.event_type == feedback.MOUSE_DOWN:
            # since default marker orientation stays fixed, change in orientation
            # must be applied relative to reference orientation when we started dragging
            self.x_target = self.x_current
            self.ee_tracking = 1
        elif feedback.event_type == feedback.MOUSE_UP:
            self.ee_tracking = 0
        elif feedback.event_type == feedback.POSE_UPDATE:
            self.mutex.acquire()
            self.x_target = feedback.pose
            self.mutex.release()

    def timer_callback(self, event):
        if not self.cc_mode:
            return
        self.mutex.acquire()
        if self.ee_tracking:
            self.pub_command.publish(self.x_target)
        self.mutex.release()


if __name__ == '__main__':
    rospy.init_node('cartesian_control', anonymous=True)
    print "Initializing Marker control"
    config_file = sys.argv[1]
    config_data = ConfigParser.ConfigParser()
    config_data.read(config_file)
    mc = MarkerControl(config_data, 'marker_control')
    rospy.sleep(1.0)
    print "Ready"
    rospy.spin()