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


class DoublePendulumLearningReward(IReinforcementLearningRewardFunction):

    def __init__(self,
                 max_cart_position,
                 max_first_pole_angle,
                 max_second_pole_angle,
                 terminate_reward):
        IReinforcementLearningRewardFunction.__init__(self)
        
        self.max_cart_position = max_cart_position
        self.max_first_pole_angle = max_first_pole_angle
        self.max_second_pole_angle = max_second_pole_angle
        self.terminate_reward = terminate_reward
        self.start_time = time.time()

    def get_reward(self, state_msg):
        poles_terminate = self.terminate_poles_angles(state_msg)
        cart_terminate = self.terminate_cart_state(state_msg)
        terminate_reward = self.terminate_reward if poles_terminate or cart_terminate else 0
        time_reward = self.get_time_reward()
        return terminate_reward + time_reward

    def terminate_state(self, state_msg):
        cart_terminate = self.terminate_cart_state(state_msg)
        poles_terminate = self.terminate_poles_angles(state_msg)
        
        terminate = cart_terminate or poles_terminate
        if terminate:
            self.terminate()
            self.start_time = time.time()

        return terminate
    
    def get_time_reward(self):
        return time.time() - self.start_time
        
    def terminate_poles_angles(self, state_msg):
        angles = DoublePendulumLearningReward.get_angles(state_msg)
        fp_terminate = abs(angles["first pole"]) > self.max_first_pole_angle
        sp_terminate = abs(angles["second pole"]) > self.max_second_pole_angle
        return fp_terminate or sp_terminate
        
    def terminate_cart_state(self, state_msg):
        return abs(state_msg.cart_state.pose.position.x) > self.max_cart_position

    @staticmethod
    def get_angles(state_msg):

        fp_pitch = DoublePendulumLearningReward.get_rpy_from_quaternion(
            state_msg.first_pole.pose.rotation)[0]
        sp_pitch = DoublePendulumLearningReward.get_rpy_from_quaternion(
            state_msg.second_pole.pose.rotation)[0]
        
        rospy.logdebug("first pole pitch angle: %f", fp_pitch, logger_name="double_pendulum_rl")
        rospy.logdebug("second pole pitch angle: %f", sp_pitch, logger_name="double_pendulum_rl")
        
        return {"first pole": fp_pitch,"second pole" : sp_pitch}
    
    @staticmethod
    def get_rpy_from_quaternion(quaternion):
        return  euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])