#!/usr/bin/env python

import rospy
import roslib
import tf
import PyKDL
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Int32
from tf_conversions import posemath as PoseMath
import os
import ConfigParser
import ast
import sys
# remove the following debugging util after finishing
import ipdb
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import Marker

def get_tf_camera2optical():
    rot = PyKDL.Rotation(0,0,1,-1,0,0,0,-1,0)
    pos = PyKDL.Vector(0,15.0/1000,-1.5/1000)
    frame = PyKDL.Frame(rot,pos)
    return frame

def read_config_file(config_file_name):
    config_data = ConfigParser.ConfigParser()
    path = os.path.join(os.environ['ARM_TELEOP_UR5_SRC'],'teleop/config/',config_file_name)
    config_data.read(path)
    return config_data

def convert_PyKDL_to_geometry_msgs_pose(frame_PyKDL):
    pose = Pose()
    pose.position.x = frame_PyKDL.p.x()
    pose.position.y = frame_PyKDL.p.y()
    pose.position.z = frame_PyKDL.p.z()
    quat = frame_PyKDL.M.GetQuaternion()
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    return pose

class CameraFrameCalibration:
    def __init__(self, config_file_name):
        # read the config file
        config_data = read_config_file(config_file_name)
        rospy.init_node('camera_tf')
        self.correcting = False
        self.camera_frame_correction = PyKDL.Frame.Identity()
        self.camera_frame = PyKDL.Frame.Identity()
        self.init_frames_from_config(config_data)
        self.init_marker()
        
    def init_frames_from_config(self, config_data):
        # init the ros rate
        self.rate = rospy.Rate(config_data.getfloat(
            'camera','rate_Hz'))
        # frame
        quat = ast.literal_eval(config_data.get(
            'camera','TF_camera2ee_rot'))
        trans = ast.literal_eval(config_data.get(
            'camera','TF_camera2ee_trans'))
        frame = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(quat[0],quat[1],quat[2],quat[3]),
            PyKDL.Vector(trans[0],trans[1],trans[2]))
        self.TF_camera2ee = frame
        # subscriber to robot ee frame
        self.robot_ee_frame_received = False
        rostopic_robot_ee = config_data.get(
            'camera', 'rostopic_ee_frame')
        self.robot_ee_frame = PyKDL.Frame.Identity()
        rospy.Subscriber(rostopic_robot_ee, PoseStamped, self.robot_ee_frame_CB)
        # subscriber to object pose estimator
        rostopic_obj_pose_est = config_data.get(
            'object_perception', 'rostopic_obj_pose_in_cam')
        self.obj_pose_in_cam = None
        self.obj_pose_in_robot_base = None
        rospy.Subscriber(rostopic_obj_pose_est, PoseStamped, self.obj_pose_est_CB)
        # publisher to give command to the robot
        rostopic_robot_command = config_data.get(
            'robot_command', 'rostopic_robot_command')
        self.pub_robot_command = rospy.Publisher(rostopic_robot_command, Pose, queue_size=1)
        # subscriber to the flab of enabling the auto grasping
        rostopic_robot_command_enable = config_data.get(
            'robot_command', 'rostopic_robot_command_enable')
        self.enable_robot_command = False
        rospy.Subscriber(rostopic_robot_command_enable, Int32, self.enable_robot_command_CB)
        # get hand offset and default pickup orientation from object pose
        object_offset = ast.literal_eval(config_data.get(
            'robot_command', 'object_offset'))
        self.object_offset = PyKDL.Vector(*object_offset)
        default_pickup_orientation = ast.literal_eval(config_data.get(
            'robot_command', 'default_pickup_orientation'))
        self.default_pickup_orientation = PyKDL.Rotation.Quaternion(*default_pickup_orientation)

    def init_marker(self):

        self.marker_server = InteractiveMarkerServer("correction_markers")

        control_marker = InteractiveMarker()
        # control_marker.header.frame_id = self.robot.get_root()
        control_marker.header.frame_id = "base"
        control_marker.name = "corr_marker"

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

        control_marker.scale = 0.25
        self.marker_server.insert(control_marker, self.correction_marker_feedback)

        # 'commit' changes and send to all clients
        self.marker_server.applyChanges()

    def correction_marker_feedback(self, feedback):
        if feedback.event_type == feedback.POSE_UPDATE:
            self.camera_frame_correction = self.robot_ee_frame.Inverse()*PoseMath.fromMsg(feedback.pose)
            self.correcting=True
        else:
            self.correcting=False

    def run(self):
        while not rospy.is_shutdown():
            if self.compute_camera_frame():
                self.send_camera_tf()
                self.send_robot_command()
            self.rate.sleep()

    def send_robot_command(self):
        if self.enable_robot_command:
            # compute the robot command
            robot_command = self.obj_pose_in_robot_base
            robot_command.p = robot_command.p + self.object_offset
            robot_command.M = self.default_pickup_orientation
            # test by sending the TF
            br = tf.TransformBroadcaster()
            br.sendTransform(
                (robot_command.p.x(),robot_command.p.y(),robot_command.p.z()),
                robot_command.M.GetQuaternion(),
                rospy.Time.now(),
                "command_hand_pose",
                "base")
            # send the robot command
            self.pub_robot_command.publish(convert_PyKDL_to_geometry_msgs_pose(robot_command))
            self.enable_robot_command=False

    def enable_robot_command_CB(self, msg):
        self.enable_robot_command = msg.data

    def compute_camera_frame(self):
        if self.robot_ee_frame_received:
            # [camera_frame_correction] is captured in robot base
            TF_camera2ee = self.camera_frame_correction*self.TF_camera2ee
            self.camera_frame = self.robot_ee_frame * TF_camera2ee
            if self.correcting:
                quat = TF_camera2ee.M.GetQuaternion()
                pos = TF_camera2ee.p
                print('quat: [{0}, {1}, {2}, {3}], pos: [{4}, {5}, {6}]'.format(
                    round(quat[0],3), round(quat[1],3), round(quat[2],3), round(quat[3],3),
                    round(pos.x(),3), round(pos.y(),3), round(pos.z(),3)))
            return True
        else:
            return False

    def send_camera_tf(self):
        # self.camera_frame is the optical frame of the camera
        tf_camera2optical = get_tf_camera2optical()
        camera_link_frame = self.camera_frame*tf_camera2optical.Inverse()
        br = tf.TransformBroadcaster()
        br.sendTransform(
            (camera_link_frame.p.x(),camera_link_frame.p.y(),camera_link_frame.p.z()),
            camera_link_frame.M.GetQuaternion(),
            rospy.Time.now(),
            "camera_link",
            "base")

        if self.obj_pose_in_robot_base is not None:
            br.sendTransform(
                (self.obj_pose_in_robot_base.p.x(),self.obj_pose_in_robot_base.p.y(),self.obj_pose_in_robot_base.p.z()),
                self.obj_pose_in_robot_base.M.GetQuaternion(),
                rospy.Time.now(),
                "obj_pose_est",
                "base")



    def robot_ee_frame_CB(self, msg_data):
        self.robot_ee_frame = PoseMath.fromMsg(msg_data.pose)
        if self.robot_ee_frame_received:
            pass
        else:
            self.update_marker(msg_data.pose)
            self.robot_ee_frame_received = True

    def obj_pose_est_CB(self, msg_data):
        self.obj_pose_in_cam = PoseMath.fromMsg(msg_data.pose)
        if self.compute_camera_frame():
            self.obj_pose_in_robot_base = self.camera_frame * self.obj_pose_in_cam 



    def update_marker(self, msg_data_pose):
        self.marker_server.setPose("corr_marker", msg_data_pose)
        self.marker_server.applyChanges()

if __name__ == '__main__':
    try:
        cam_tf = CameraFrameCalibration(sys.argv[1])
        cam_tf.run()
    except rospy.ROSInterruptException:
        pass