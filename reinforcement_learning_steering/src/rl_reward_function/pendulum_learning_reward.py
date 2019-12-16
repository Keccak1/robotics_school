import copy
import time

import numpy as np
import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates
from simulations_messages.msg import DoublePendulumStates

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from ireinforcement_learning_reward_function import IReinforcementLearningRewardFunction


class PendulumLearningReward(IReinforcementLearningRewardFunction):

    def __init__(self,
                 max_cart_position,
                 max_pole_angle,
                 terminate_reward,
                 time_factor = 1):
        IReinforcementLearningRewardFunction.__init__(self)
        
        self.max_cart_position = max_cart_position
        self.max_pole_angle = max_pole_angle
        self.terminate_reward = terminate_reward
        self.time_factor = time_factor
        self.start_time = time.time()

    def get_reward(self, state_msg):
        pole_terminate = self.terminate_pole_angle(state_msg)
        cart_terminate = self.terminate_cart_state(state_msg)
        terminate_reward = self.terminate_reward if pole_terminate or cart_terminate else 0
        time_reward = self.get_time_reward()
        return terminate_reward + time_reward

    def terminate_state(self, state_msg):
        cart_terminate = self.terminate_cart_state(state_msg)
        pole_terminate = self.terminate_pole_angle(state_msg)
        
        terminate = cart_terminate or pole_terminate
        if terminate:
            self.terminate()
            self.start_time = time.time()

        return terminate
    
    def get_time_reward(self):
        return time.time() - self.start_time
        
    def terminate_pole_angle(self, state_msg):
        angle = PendulumLearningReward.get_angle(state_msg)
        fp_terminate = abs(angle) > self.max_pole_angle
        return fp_terminate
        
    def terminate_cart_state(self, state_msg):
        return abs(state_msg.cart_state.pose.position.x) > self.max_cart_position

    @staticmethod
    def get_angle(state_msg):

        fp_pitch = PendulumLearningReward.get_rpy_from_quaternion(
            state_msg.first_pole.pose.rotation)[0]

        rospy.logdebug("first pole pitch angle: %f", fp_pitch, logger_name="double_pendulum_rl")
        
        return fp_pitch
    
    @staticmethod
    def get_rpy_from_quaternion(quaternion):
        return  euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])