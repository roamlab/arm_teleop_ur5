#!/usr/bin/env python
from teleop.utils import RegistryMeta
import numpy as np


class CommandFilterBaseClass(object):
    __metaclass__ = RegistryMeta

    def __init__(self, config_data, section_name):
        self.previous_command = None
        self.filter_setup_complete = False
        self.num_past_samples = config_data.getint(section_name, 'num_past_samples')
        self.command_storage_array = None
        self.setup_iter = 0

    def setup_filter(self, command):
        """

        Args:
            command: numpy array containing a command

        Returns:

        """
        if self.command_storage_array is None:
            # if storage array hasn't been created, do it now
            # need to wait until you get the first command so you know the dimensions of a command
            # print('command', command)
            # print('self.num_past_samples', self.num_past_samples)
            self.command_storage_array = np.zeros((self.num_past_samples, len(command)))
        self.update_command_storage(command)
        self.setup_iter += 1
        if self.setup_iter == self.num_past_samples:
            # setup isn't complete until you fill up the storage
            self.filter_setup_complete = True

    def update_command_storage(self, command):
        """
        self.command_storage_array is a FIFO array.
        For each new command, remove the top command and append the newest command
        Args:
            command: any sequence that holds the command

        """
        # print('pre roll', self.command_storage_array)

        self.command_storage_array = np.roll(self.command_storage_array, shift=1, axis=0)
        # print('post_roll', self.command_storage_array)
        # print('command array shape', self.command_storage_array.shape)
        self.command_storage_array[0, :] = command

