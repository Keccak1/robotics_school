#!/usr/bin/env python
import rospy
from rl_environment.reinforcement_learning_environment import ReinforcementEnvironment
from rl_reward_function.differential_model_learning_reward import DifferentialModelLearningReward
from rl_steering_algorithm.diff_model_steering_algorithm import DiffModelSteering
from gazebo_msgs.msg import ModelStates

if __name__ == "__main__":
    rospy.init_node('rl_diff_model')
    reward_function = DifferentialModelLearningReward(terminate_time=10,
                                                      beer_moved_reward=300,
                                                      max_distance=10,
                                                      collision_min_distance=0.3,
                                                      distance_factor=0.2, time_factor=0.005)

    steering_algorithm = DiffModelSteering()

    dp_rl = ReinforcementEnvironment(reward_function=reward_function,
                                     steering_algorithm=steering_algorithm,
                                     state_msg=ModelStates,
                                     robot_namespace="/differential_drive_model",
                                     state_topic="/gazebo/model_states",
                                     steering_topic="/differential_drive_model/differential_drive_controller/cmd_vel"
                                     )

    rospy.spin()
