import ast
import numpy as np


class FrameAdapter(object):
    def __init__(self, config_data, section_name):
        self.from_fixed_frame = config_data.get(section_name, 'from_fixed_frame')
        self.to_fixed_frame = config_data.get(section_name, 'to_fixed_frame')

        self.from_moving_frame = config_data.get(section_name, 'from_moving_frame')
        self.to_moving_frame = config_data.get(section_name, 'to_moving_frame')

        self.to_fix_T_from_fix = np.asarray(ast.literal_eval(config_data.get(section_name, 'to_fix_T_from_fix')))
        self.to_mf_T_from_mf = np.asarray(ast.literal_eval(config_data.get(section_name, 'to_mf_T_from_mf')))

    def change_fixed_frame(self, transform):
        return np.dot(self.to_fix_T_from_fix, transform)

    def change_moving_frame(self, transform):
        return np.dot(transform, self.to_mf_T_from_mf)

    def check_adapter(self, from_fixed, to_fixed, from_mf, to_mf):
        assert from_fixed == self.from_fixed_frame
        assert to_fixed == self.to_fixed_frame
        assert from_mf == self.from_moving_frame
        assert to_mf == self.to_moving_frame