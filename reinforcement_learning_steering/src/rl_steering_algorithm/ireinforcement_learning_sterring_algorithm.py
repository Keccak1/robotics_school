from abc import ABCMeta, abstractmethod
import rospy
from reinforcement_learning_steering.msg import RLEpisodeState


class IReinforcementLearningSteeringAlgorithm():
    __metaclass__ = ABCMeta

    def __init__(self):
        self.start = True
    
    def prepare_next_episode(self, terminate, current_state):
        if terminate:
            self.reset_simulation()
        else:
            self.last_state = current_state
            
            
    @abstractmethod
    def reset_simulation(self):
        pass

    @abstractmethod
    def get_steering_msg(self):
        pass

    @abstractmethod
    def steer(self, state_msg, reward, terminate):
        pass
