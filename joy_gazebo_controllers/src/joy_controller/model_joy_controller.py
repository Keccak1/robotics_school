import rospy
from sensor_msgs.msg import Joy

class JoyController:
    def __init__(self, from_joy_msg, topic_name_pub, topic_name_sub, rate=10, queue_size_sub=10, queue_size_pub=10):
        
        self.from_joy_msg = from_joy_msg
        self.sub = rospy.Subscriber(topic_name_sub, Joy, self.update, queue_size=queue_size_sub)
        self.pub = rospy.Publisher(topic_name_pub, self.from_joy_msg.get_ros_msg().__class__ , queue_size=queue_size_pub)
        
    def update(self, joy_msg):
        self.from_joy_msg.update(joy_msg)
        self.pub.publish(self.from_joy_msg.get_ros_msg())
        
            
            
        
        