import ifrom_joy_msg as Ijoy
from geometry_msgs.msg import Twist
import rospy


class CmdVelFromJoyMsg(Ijoy.JoyMsgToRosMsg):

    def __init__(self, max_vel_x, max_vel_y, max_vel_yaw):
        Ijoy.JoyMsgToRosMsg.__init__(self)
        self.max_vel_x = max_vel_x
        self.max_vel_y = max_vel_y
        self.max_vel_yaw = max_vel_yaw
        self.twist = Twist()

    def update(self, joy_msg):
        self.twist.linear.x = self.max_vel_x*joy_msg.axes[1]
        self.twist.linear.y = self.max_vel_y*joy_msg.axes[0]
        self.twist.angular.z = self.max_vel_yaw*joy_msg.axes[3]

    def get_ros_msg(self):
        rospy.logdebug("Joy steering msg x: %f", self.twist.linear.x, logger_name="joy_cmd_vel")
        rospy.logdebug("Joy steering msg y: %f", self.twist.linear.y,logger_name="joy_cmd_vel")
        rospy.logdebug("Joy steering msg yaw: %f", self.twist.angular.z, logger_name="joy_cmd_vel")
        return self.twist
