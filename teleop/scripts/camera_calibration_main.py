import rospy
import sys
from teleop import CameraFrameCalibration

if __name__ == '__main__':
    try:
        cam_tf = CameraFrameCalibration(sys.argv[1])
        cam_tf.run()
    except rospy.ROSInterruptException:
        pass