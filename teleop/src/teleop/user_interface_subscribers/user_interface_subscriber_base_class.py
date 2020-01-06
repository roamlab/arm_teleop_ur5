import time
from teleop.utils import RegistryMeta
from threading import Lock


class UserInterfaceSubscriberBaseClass(object):
    __metaclass__ = RegistryMeta

    def __init__(self, config_data, section_name):
        self.mutex = Lock()
        self.cmd = []
        self.cmd_time = time.time()
        self.rostopic_subscriber_cmd = config_data.get(section_name, 'rostopic_subscriber_cmd')

    def cmd_CB(self, msg_data):
        #self.mutex.acquire()
        self.cmd = msg_data
        self.cmd_time = time.time()
        # print("cmd_time", self.cmd_time)
        #self.mutex.release()

