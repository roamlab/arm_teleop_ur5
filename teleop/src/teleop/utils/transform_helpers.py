import numpy
import tf
from geometry_msgs.msg import Pose, PoseStamped, Twist, TwistStamped


def convert_pose_to_transform(p):
    trans = tf.transformations.translation_matrix((p.position.x,
                                                  p.position.y,
                                                  p.position.z))
    rot = tf.transformations.quaternion_matrix((p.orientation.x,
                                                p.orientation.y,
                                                p.orientation.z,
                                                p.orientation.w))
    return numpy.dot(trans, rot)


def convert_transform_to_pose(T):
    t = Pose()
    position = tf.transformations.translation_from_matrix(T)
    orientation = tf.transformations.quaternion_from_matrix(T)
    t.position.x = position[0]
    t.position.y = position[1]
    t.position.z = position[2]
    t.orientation.x = orientation[0]
    t.orientation.y = orientation[1]
    t.orientation.z = orientation[2]
    t.orientation.w = orientation[3]
    return t


def convert_PyKDL_twist_to_geometry_msgs_twist(twist_PyKDL):
    twist = Twist()
    twist.linear.x = twist_PyKDL.vel.x()
    twist.linear.y = twist_PyKDL.vel.y()
    twist.linear.z = twist_PyKDL.vel.z()
    twist.angular.x = twist_PyKDL.rot.x()
    twist.angular.y = twist_PyKDL.rot.y()
    twist.angular.z = twist_PyKDL.rot.z()
    return twist


def convert_twist_to_numpy_array(twist):
    twist_array = numpy.zeros(6, dtype=float)
    twist_array[0] = twist.linear.x
    twist_array[1] = twist.linear.y
    twist_array[2] = twist.linear.z
    twist_array[3] = twist.angular.x
    twist_array[4] = twist.angular.y
    twist_array[5] = twist.angular.z
    return twist_array


def convert_numpy_array_to_twist(twist_array):
    twist = Twist()
    twist.linear.x = twist_array[0]
    twist.linear.y = twist_array[1]
    twist.linear.z = twist_array[2]

    twist.angular.x = twist_array[3]
    twist.angular.y = twist_array[4]
    twist.angular.z = twist_array[5]
    return twist
