import tf
import numpy as np


def get_fixed_robot_transforms(robot):
    """
    gets fixed transforms for all links in robot
    From Matei's Intro to Robotics Notes: Kinematics.pdf
    T_li = fixed transforms. Do not change at run-time. They reflect the physical dimensions of the robot links (L_i)
    T_ji(q_i) = variable transforms. They reflect the joint (J_i) values of the robot at any given time
    * note that J_i moves L_i

    b_T_ee = T_lb*(/PRODUCT_(i=0)^(n-1))[T_ji*T_li(qi)]
    b_T_ee = T_lb*[T_j0(q0)*T_l0]*[T_j1(q1)*T_l1]*[T_j2(q2)*T_l2]

    Args:
        robot: a

    Returns:
        fixed_transforms which is a list of T_li

    """
    fixed_transforms = []
    link = robot.get_root()

    while True:
        if link not in robot.child_map:
            break

        (joint_name, next_link) = robot.child_map[link][0]
        joint = robot.joint_map[joint_name]

        R_l = tf.transformations.euler_matrix(joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2])
        t_l = tf.transformations.translation_matrix(joint.origin.xyz)
        T_l = np.dot(t_l, R_l)
        fixed_transforms.append(T_l)

        link = next_link

    return fixed_transforms