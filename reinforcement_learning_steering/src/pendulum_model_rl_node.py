#!/usr/bin/env python
import rospy

from simulations_messages.msg import DoublePendulumStates
from simulations_messages.msg import Vector3f

from rl_steering_algorithm.pendulum_steering_algorithm import PendulumSteering
from rl_reward_function.pendulum_learning_reward import PendulumLearningReward
from rl_environment.reinforcement_learning_environment import ReinforcementEnvironment


if __name__ == "__main__":

    rospy.init_node('rl__pendulum')
    reward_function = PendulumLearningReward(max_cart_position=1.5,
                                                   max_pole_angle=0.4,
                                                   time_factor=2,
                                                   terminate_reward=-1000)

    steering_algorithm = PendulumSteering()

    dp_rl = ReinforcementEnvironment(reward_function,
                                     steering_algorithm,
                                     DoublePendulumStates,
                                     "/double_pendulum",
                                     "/double_pendulum_states",
                                     "/double_pendulum_force"
                                     )

    rospy.spin()
