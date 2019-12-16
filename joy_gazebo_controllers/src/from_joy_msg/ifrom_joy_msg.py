from abc import ABCMeta, abstractmethod
from sensor_msgs.msg import Joy


class JoyMsgToRosMsg:
    __metaclass__ = ABCMeta

    @abstractmethod
    def update(self, joy_msg):
        pass

    @abstractmethod
    def get_ros_msg(self):
        pass
