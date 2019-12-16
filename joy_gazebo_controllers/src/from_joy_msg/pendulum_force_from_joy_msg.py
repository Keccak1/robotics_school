import ifrom_joy_msg as Ijoy
from simulations_messages.msg import Vector3f
import rospy
class PendulumForceFromJouMsg(Ijoy.JoyMsgToRosMsg):

    def __init__(self, max_force):
        Ijoy.JoyMsgToRosMsg.__init__(self)
        self.force = Vector3f()
        self.max_force = max_force

    def update(self, joy_msg):
        self.force.x = joy_msg.axes[0] * self.max_force
        
    def get_ros_msg(self):
        rospy.logdebug("Joy steering msg x: %f", self.force.x, logger_name="joy_pendulum_force")
        return self.force

