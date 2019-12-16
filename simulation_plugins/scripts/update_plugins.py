#!/usr/bin/env python

import rospy
import plugin_manager


if __name__ == "__main__":
    try:
        rospy.init_node('update_plugin', anonymous=True)
        manager = plugin_manager.PluginManager()
        manager.update_plugins()

    except rospy.ROSInterruptException as rose:
        print(rose)

    except FileNotFoundError as fe:
        print(fe)
        


