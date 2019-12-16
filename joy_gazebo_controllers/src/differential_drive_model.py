#!/usr/bin/env python

import rospy
from joy_controller import model_joy_controller
from from_joy_msg import cmd_vel_from_joy_msg

if __name__ == "__main__":

    try:
        rospy.init_node("joy_formula_diff")
        cmd_controll = cmd_vel_from_joy_msg.CmdVelFromJoyMsg(1,1,1)
        joy = model_joy_controller.JoyController(cmd_controll,
                                                 "/differential_drive_model/differential_drive_controller/cmd_vel",
                                                 "/joy")
        rospy.spin()
    except rospy.ROSInterruptException as roserr:
        print(roserr)
