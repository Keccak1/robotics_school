#!/usr/bin/env python

import rospy
from joy_controller import model_joy_controller
from from_joy_msg import pendulum_force_from_joy_msg


if __name__ == "__main__":

    try:
        rospy.init_node("pendulum_joy_controller")
        force_controll = pendulum_force_from_joy_msg.PendulumForceFromJouMsg(
            100)
        joy = model_joy_controller.JoyController(force_controll,
                                                 "/double_pendulum_force",
                                                 "/joy")
        rospy.spin()

    except rospy.ROSInterruptException as roserr:
        print(roserr)
