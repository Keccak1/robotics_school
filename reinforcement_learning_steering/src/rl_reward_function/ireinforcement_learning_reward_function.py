import copy
from abc import ABCMeta, abstractmethod
import rospy
from reinforcement_learning_steering.msg import RLEpisodeState
import copy

class IReinforcementLearningRewardFunction:
    __metaclass__ = ABCMeta

    def __init__(self):
        self.episode_state = RLEpisodeState()
        self.reset_episode_rl_state()

    def update(self, state_msg):
        reward = self.get_reward(state_msg)
        term = self.terminate_state(state_msg)
        self.update_episode_rl_state(reward)
        episode_state = copy.deepcopy(self.episode_state)
        if term:
            self.reset_episode_rl_state()
        return term, episode_state
    
    def update_episode_rl_state(self, reward):
        self.episode_state.iteration += 1
        self.episode_state.total_reward += reward
        self.episode_state.last_reward = reward

    @abstractmethod
    def get_reward(self, state_msg):
        pass

    @abstractmethod
    def terminate_state(self, state_msg):
        pass

    def reset_episode_rl_state(self):
        self.episode_state.iteration = 0
        self.episode_state.total_reward = 0
        self.episode_state.last_reward = 0
