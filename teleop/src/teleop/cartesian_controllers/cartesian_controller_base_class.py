#!/usr/bin/env python
from teleop.utils import RegistryMeta


class CartesianControllerBaseClass(object):
    __metaclass__ = RegistryMeta

    def __init__(self, config_data, section_name):
        pass

    def compute_vee(self, current_pose, desired_pose):
        raise NotImplementedError

    def compute_qd(self, v_ee, joint_transforms, fixed_T_tool_current, joint_axes=None):
        raise NotImplementedError
