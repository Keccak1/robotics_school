import copy
from abc import ABCMeta, abstractmethod
import rospy
from reinforcement_learning_steering.msg import RLEpisodeState
from std_srvs.srv import Empty


class IReinforcementLearningRewardFunction:
    __metaclass__ = ABCMeta

    def __init__(self):
        self.current_state = RLEpisodeState()
        self.reset_state()

    def update(self, state_msg):
        reward = self.get_reward(state_msg)
        term = self.terminate_state(state_msg)
        self.update_state(reward)
        state = self.return_state()
        if term:
            self.reset_state()
        self.current_state.iteration += 1
        return term, state
    
    def update_state(self, reward):
        self.current_state.total_reward += reward
        self.current_state.last_reward = reward
        
    def return_state(self):
        state = RLEpisodeState()
        state.iteration = self.current_state.iteration
        state.total_reward = self.current_state.total_reward
        state.last_reward = self.current_state.last_reward
        return state

    @abstractmethod
    def get_reward(self, state_msg):
        pass

    @abstractmethod
    def terminate_state(self, state_msg):
        pass

    def reset_state(self):
        self.current_state.iteration = 0
        self.current_state.total_reward = 0
        self.current_state.last_reward = 0

    @staticmethod
    def terminate():
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            reset_sim = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
            resp1 = reset_sim()
        except rospy.ServiceException, e:
            print("Cannot restart simulation")

