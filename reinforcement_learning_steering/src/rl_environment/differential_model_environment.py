import random

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import time

from ireinforcement_learning_environment import IReinforcementLearningEnvironment


class DifferentialModelEnvironment(IReinforcementLearningEnvironment):

    def __init__(self,
                 reward_function,
                 steering_algorithm,
                 state_msg,
                 robot_namespace,
                 state_topic,
                 steering_topic):

        IReinforcementLearningEnvironment.__init__(self,
                                                   reward_function,
                                                   steering_algorithm,
                                                   state_msg,
                                                   robot_namespace,
                                                   state_topic,
                                                   steering_topic)
        self.change_beer_position()

    def prepare_new_episode(self):
        self.pause_simulation()
        # print 'pause'
        # time.sleep(1.0)
        IReinforcementLearningEnvironment.reset_simulation()
        # print 'reset sim'
        # time.sleep(1.0)
        self.change_beer_position()
        # print 'change'
        time.sleep(2.0)
        self.unpause_simulation()
        print
        self.rl_state.episode_iteration += 1
        self.rl_state.current_episode.iteration = -1
        self.rl_state.current_episode.last_reward = 0
        self.rl_state.current_episode.total_reward = 0
        self.rl_state.terminate = False

    def change_beer_position(self):
        beer_state = ModelState()
        beer_state.model_name = 'beer'
        beer_state.pose.position.x, beer_state.pose.position.y = DifferentialModelEnvironment.generate_beer_position()
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            response = set_model_state(beer_state)
        except rospy.ServiceException, e:
            print("Cannot set model state in simulation")

    @staticmethod
    def generate_beer_position():
        max_dist = 5
        min_dist = 1
        # Randomly generate 2 numbers from [-max_dist, max_dist)
        x, y = random.random() * 2 * max_dist - max_dist, random.random() * 2 * max_dist - max_dist
        # Make sure to spawn beer not to close to beer or on it.
        if 0 <= x < min_dist:
            x = min_dist
        elif -min_dist < x <= 0:
            x = -min_dist
        if 0 <= y < min_dist:
            y = min_dist
        elif -min_dist < y <= 0:
            y = -min_dist
        # return x, y
        return 1, 0
