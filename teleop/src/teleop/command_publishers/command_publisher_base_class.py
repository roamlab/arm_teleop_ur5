from teleop.utils import RegistryMeta
from threading import Lock


class CommandPublisherBaseClass(object):
    __metaclass__ = RegistryMeta

    def __init__(self, config_data, section_name):
        self.mutex = Lock()
        self.rostopic_publisher_cmd = config_data.get(section_name, 'rostopic_publisher_cmd')
        self.pub = None

    def publish(self, cmd):
        self.mutex.acquire()
        self.pub.publish(cmd)
        self.mutex.release()

    def get_zero_command(self):
        raise NotImplementedError






