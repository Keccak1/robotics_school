import copy
import numpy as np

import rospy
from ireinforcement_learning_sterring_algorithm import IReinforcementLearningSteeringAlgorithm

from geometry_msgs.msg import Twist
from ddpg_implementation import ddpg_agent


class DiffModelSteering(IReinforcementLearningSteeringAlgorithm):

    def __init__(self):
        IReinforcementLearningSteeringAlgorithm.__init__(self)
        self.steering_msg = Twist()
        self.controller = ddpg_agent.Agent(4, 3, 0)
        self.last_state = np.zeros(4, dtype=np.float32)

    def get_steering_msg(self):
        twist = Twist()
        twist.linear.x = self.steering_msg.linear.x
        twist.linear.y = self.steering_msg.linear.y
        twist.angular.z = self.steering_msg.angular.z
        return twist

    def steer(self, state_msg, reward, terminate):
        current_state = DiffModelSteering.get_network_input(state_msg)
        action = self.controller.act(current_state)
        DiffModelSteering.log_action(action)
        self.update_steering_msg(action)
        self.step(action, reward, current_state, terminate)
        self.prepare_next_episode(terminate, current_state)
        return self.get_steering_msg()

    def step(self, action, reward, current_state,terminate):
        if self.start:
            self.start = False
        else:
            self.controller.step(copy.deepcopy(self.last_state), action, np.array(reward.last_reward, dtype=np.float32),
                                 current_state, np.bool(terminate))

    def reset_simulation(self):
        self.start = True
        self.last_state = np.zeros(4, dtype=np.float32)
        
    def update_steering_msg(self, action):
        self.steering_msg.linear.x = action[0]
        self.steering_msg.linear.y = action[1]
        self.steering_msg.angular.z = action[2]

    @staticmethod
    def get_network_input(state_msg):
        indexes = DiffModelSteering.get_indexes(state_msg)
        model_x = state_msg.pose[indexes["model"]].position.x
        model_y = state_msg.pose[indexes["model"]].position.y
        beer_x = state_msg.pose[indexes["beer"]].position.x
        beer_y = state_msg.pose[indexes["beer"]].position.y
        return np.array((model_x, model_y, beer_x, beer_y), dtype=np.float32)

    @staticmethod
    def get_indexes(state_msg):
        beer_index = state_msg.name.index("beer")
        model_index = state_msg.name.index("differential_drive_model")
        return {"beer": beer_index, "model": model_index}

    @staticmethod
    def log_action(action):
        rospy.logdebug("action x: %f", action[0],
                       logger_name="differential_drive_rl")
        rospy.logdebug("action y: %f", action[1],
                       logger_name="differential_drive_rl")
        rospy.logdebug("action yaw: %f", action[2],
                       logger_name="differential_drive_rl")
