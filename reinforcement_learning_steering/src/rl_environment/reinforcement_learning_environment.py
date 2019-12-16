import rospy

from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelStates

from reinforcement_learning_steering.msg import RLSimulation


class ReinforcementEnvironment(object):

    def __init__(self,
                 reward_function,
                 steering_algorithm,
                 state_msg,
                 robot_namespace,
                 state_topic,
                 steering_topic):

        self.reward_function = reward_function
        self.steering_algorithm = steering_algorithm
        self.rl_state = RLSimulation()
        self.state_sub = rospy.Subscriber(
            state_topic, state_msg, self.rl_callback, queue_size=1)
        self.steer_pub = rospy.Publisher(
            steering_topic, steering_algorithm.get_steering_msg().__class__, queue_size=1)
        self.rl_pub = rospy.Publisher(robot_namespace + "/rl", RLSimulation, queue_size=1)

    def rl_callback(self, state_msg):
        terminate, rl_state = self.reward_function.update(state_msg)
        steering_msg = self.steering_algorithm.steer(state_msg, rl_state, terminate)
        self.steer_pub.publish(steering_msg)
        self.update_rl_state(rl_state, terminate)
        self.rl_pub.publish(self.return_rl_state())

        if terminate:
            self.prepare_new_episode()
            
    def update_rl_state(self, reward, terminate):
        self.rl_state.current_episode = reward
        self.rl_state.terminate = terminate

    def prepare_new_episode(self):
        self.rl_state.episode_iteration += 1
        self.rl_state.current_episode.iteration = -1
        self.rl_state.current_episode.last_reward = 0
        self.rl_state.current_episode.total_reward = 0
        self.rl_state.terminate = False

    def return_rl_state(self):
        state = RLSimulation()
        state.current_episode = self.rl_state.current_episode
        state.episode_iteration = self.rl_state.episode_iteration
        state.terminate = self.rl_state.terminate
        return state

