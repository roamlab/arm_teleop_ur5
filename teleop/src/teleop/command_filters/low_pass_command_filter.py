import numpy as np
from teleop.command_filters import CommandFilterBaseClass


class LowPassCommandFilter(CommandFilterBaseClass):
    def __init__(self, config_data, section_name):
        super(LowPassCommandFilter, self).__init__(config_data, section_name)
        self.difference_limit_value = config_data.getfloat(section_name, 'difference_limit_value')

    def filter_command(self, command):
        if not self.filter_setup_complete:
            self.setup_filter(command)
            # if command storage hasn't been filled, we cannot yet filter commands
            # so we return the original command
            return command
        else:
            # filter
            average_command = np.average(self.command_storage_array, axis=0)
            # print('average_command', average_command)
            command_difference = command-average_command
            difference_limit = np.zeros(len(command_difference))
            difference_limit[:] = self.difference_limit_value
            if np.all(np.less(abs(command_difference), difference_limit)):
                self.update_command_storage(command)
                print('command not different enough from average, no command sent')
                return None
            else:
                self.update_command_storage(command)
                average_command = np.average(self.command_storage_array, axis=0)
                return average_command

