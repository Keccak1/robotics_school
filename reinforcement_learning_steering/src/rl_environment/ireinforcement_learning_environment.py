from abc import ABCMeta, abstractmethod

import rospy
from std_srvs.srv import Empty

from reinforcement_learning_steering.msg import RLSimulation


class IReinforcementLearningEnvironment:
    __metaclass__ = ABCMeta

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
        self.state_sub = rospy.Subscriber(state_topic, state_msg, self.rl_callback, queue_size=1)
        self.steer_pub = rospy.Publisher(steering_topic, steering_algorithm.get_steering_msg().__class__, queue_size=1)
        self.rl_pub = rospy.Publisher(robot_namespace + "/rl", RLSimulation, queue_size=1)

        self.reset_simulation()
        self.unpause_simulation()

    def rl_callback(self, state_msg):
        # self.pause_simulation()

        terminate, episode_rl_state = self.reward_function.update(state_msg)
        steering_msg = self.steering_algorithm.steer(state_msg, episode_rl_state, terminate)
        # self.steer_pub.publish(steering_msg)
        self.update_rl_state(episode_rl_state, terminate)
        self.rl_pub.publish(self.return_rl_state())
        
        # self.unpause_simulation()

        if terminate:
            self.prepare_new_episode()
            
    def update_rl_state(self, reward, terminate):
        self.rl_state.current_episode = reward
        self.rl_state.terminate = terminate

    @abstractmethod
    def prepare_new_episode(self):
        pass

    def return_rl_state(self):
        state = RLSimulation()
        state.current_episode = self.rl_state.current_episode
        state.episode_iteration = self.rl_state.episode_iteration
        state.terminate = self.rl_state.terminate
        return state

    @staticmethod
    def reset_simulation():
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            reset_sim = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
            resp1 = reset_sim()
        except rospy.ServiceException, e:
            print("Cannot restart simulation")

    @staticmethod
    def pause_simulation():
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            reset_sim = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
            resp1 = reset_sim()
        except rospy.ServiceException, e:
            print("Cannot pause simulation")

    @staticmethod
    def unpause_simulation():
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            reset_sim = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
            resp1 = reset_sim()
        except rospy.ServiceException, e:
            print("Cannot unpause simulation")
    
    # @staticmethod
    # def get_sim_time():
    #     rospy.wait_for_service('/gazebo/get_world_properties')
    #     try:
    #         get_sim_time = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
    #         resp = get_sim_time()
    #     except rospy.ServiceException, e:
    #         print("Cannot get world properties.")
    #     return resp.sim_time
