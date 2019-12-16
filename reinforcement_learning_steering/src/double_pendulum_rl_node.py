#!/usr/bin/env python
import rospy

from simulations_messages.msg import DoublePendulumStates
from simulations_messages.msg import Vector3f

from rl_steering_algorithm.double_pendulum_steering_algorithm import DoublePendulumSteering
from rl_reward_function.double_pendulum_learning_reward import DoublePendulumLearningReward
from rl_environment.reinforcement_learning_environment import ReinforcementEnvironment


if __name__ == "__main__":

    rospy.init_node('rl_doublem_pendulum')
    reward_function = DoublePendulumLearningReward(max_cart_position=1.5,
                                                   max_first_pole_angle=0.3,
                                                   max_second_pole_angle=0.3,
                                                   terminate_reward=-100)

    steering_algorithm = DoublePendulumSteering()

    dp_rl = ReinforcementEnvironment(reward_function,
                                     steering_algorithm,
                                     DoublePendulumStates,
                                     "/double_pendulum",
                                     "/double_pendulum_states",
                                     "/double_pendulum_force"
                                     )

    rospy.spin()
