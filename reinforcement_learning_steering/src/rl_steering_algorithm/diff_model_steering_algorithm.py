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
        self.observation_space_dim = 4
        self.action_space_dim = 2
        self.controller = ddpg_agent.Agent(state_size=self.observation_space_dim,
                                           action_size=self.action_space_dim,
                                           random_seed=0)
        self.last_state = np.zeros(self.observation_space_dim, dtype=np.float32)

    def get_steering_msg(self):
        twist = Twist()
        twist.linear.x = self.steering_msg.linear.x
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

    def step(self, action, reward, current_state, terminate):
        if self.start:
            self.start = False
        else:
            self.controller.step(copy.deepcopy(self.last_state), action, np.array(reward.last_reward, dtype=np.float32),
                                 current_state, np.bool(terminate))

    def reset(self):
        self.start = True
        self.last_state = np.zeros(self.observation_space_dim, dtype=np.float32)
        
    def update_steering_msg(self, action):
        self.steering_msg.linear.x = action[0]
        self.steering_msg.angular.z = action[1]

    @staticmethod
    def get_network_input(state_msg):
        indexes = DiffModelSteering.get_indexes(state_msg)
        model_x = state_msg.pose[indexes["model"]].position.x
        model_y = state_msg.pose[indexes["model"]].position.y
        beer_x = state_msg.pose[indexes["beer"]].position.x
        beer_y = state_msg.pose[indexes["beer"]].position.y
        model_vel_x = state_msg.twist[indexes["model"]].linear.x
        model_vel_ang_z = state_msg.twist[indexes["model"]].angular.z
        # print '{:f}, {:f}'.format(model_vel_x, model_vel_ang_z)
        # print '{}\n'.format(state_msg.twist[indexes["model"]])
        # print '{:f}, {:f}, {:f}, {:f}'.format(model_vel_x, model_vel_ang_z, beer_x - model_x, beer_y - model_y)
        return np.array([model_vel_x, model_vel_ang_z, beer_x - model_x, beer_y - model_y], dtype=np.float32)

    @staticmethod
    def get_indexes(state_msg):
        beer_index = state_msg.name.index("beer")
        model_index = state_msg.name.index("differential_drive_model")
        return {"beer": beer_index, "model": model_index}

    @staticmethod
    def log_action(action):
        rospy.logdebug("action x: %f", action[0],
                       logger_name="differential_drive_rl")
        # rospy.logdebug("action yaw: %f", action[1],
        #                logger_name="differential_drive_rl")
