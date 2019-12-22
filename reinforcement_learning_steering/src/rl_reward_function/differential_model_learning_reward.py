import rospy
from gazebo_msgs.srv import GetWorldProperties

from ireinforcement_learning_reward_function import IReinforcementLearningRewardFunction


class DifferentialModelLearningReward(IReinforcementLearningRewardFunction):

    def __init__(self, terminate_time,
                 beer_moved_reward,
                 max_distance,
                 collision_min_distance=0.2,
                 distance_factor=0.001,
                 time_factor=0.001):

        IReinforcementLearningRewardFunction.__init__(self)
        self.terminate_time = terminate_time
        self.beer_moved_reward = beer_moved_reward
        self.max_distance = max_distance
        self.distance_factor = distance_factor
        self.time_factor = time_factor
        self.collision_min_distance = collision_min_distance
        self.prev_dist = None

    def get_reward(self, state_msg):
        elapsed_time_reward = self.get_time_reward()
        distance_reward = self.get_distance_reward(state_msg)
        beer_moved_reward = self.get_collision_reward(state_msg)

        DifferentialModelLearningReward.log_rewards(elapsed_time_reward,
                                                    distance_reward,
                                                    beer_moved_reward)

        reward = 0
        curr_dist = DifferentialModelLearningReward.get_distance(state_msg)
        if self.collision(state_msg):
            print 'beer hit'
            reward = +100
        else:
            if self.get_elapsed_time() > self.terminate_time:
                print 'elapsed time'
                reward = -10
            else:
                if self.prev_dist:
                    reward = 0.5 * (self.prev_dist - curr_dist)
        self.prev_dist = curr_dist
        return reward

    def get_time_reward(self):
        # print 'sim time: {}'.format(self.get_elapsed_time())
        return self.get_elapsed_time() * self.time_factor

    def get_distance_reward(self, state_msg):
        return self.get_distance(state_msg) * self.distance_factor

    def get_collision_reward(self, state_msg):
        reward = self.beer_moved_reward if self.collision(state_msg) else 0
        return reward

    def terminate_state(self, state_msg):
        elapsed_time_terminate = self.get_elapsed_time_terminate()
        distance_terminate = self.get_distance_terminate(state_msg)
        collision_terminate = self.get_collision_terminate(state_msg)

        terminate = elapsed_time_terminate or distance_terminate or collision_terminate

        self.prev_dist = None if terminate else self.prev_dist
        return terminate

    def get_elapsed_time_terminate(self):
        return self.get_elapsed_time() > self.terminate_time

    def get_elapsed_time(self):
        rospy.wait_for_service('/gazebo/get_world_properties')
        try:
            get_sim_time = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
            resp = get_sim_time()
        except rospy.ServiceException, e:
            print("Cannot get world properties.")
        return resp.sim_time
    
    def get_distance_terminate(self, state_msg):
        return DifferentialModelLearningReward.get_distance(state_msg) > self.max_distance

    def get_collision_terminate(self, state_msg):
        # print('collision: {:f}'.format(self.collision(state_msg)))
        return self.collision(state_msg)

    @staticmethod
    def get_distance(state_msg):
        indexes = DifferentialModelLearningReward.get_indexes(state_msg)
        model_x = state_msg.pose[indexes["model"]].position.x
        model_y = state_msg.pose[indexes["model"]].position.y
        beer_x = state_msg.pose[indexes["beer"]].position.x
        beer_y = state_msg.pose[indexes["beer"]].position.y

        distance = ((model_x - beer_x) ** 2 + (model_y - beer_y) ** 2) ** 0.5

        return distance

    @staticmethod
    def get_indexes(state_msg):
        beer_index = state_msg.name.index("beer")
        model_index = state_msg.name.index("differential_drive_model")
        return {"beer": beer_index, "model": model_index}

    def collision(self, state_msg):
        return DifferentialModelLearningReward.get_distance(state_msg) < self.collision_min_distance

    @staticmethod
    def log_rewards(elapsed_time_reward, distance_reward, beer_moved_reward):
        rospy.logdebug("elapsed_time_reward: %f", elapsed_time_reward,
                       logger_name="differential_drive_rl")
        rospy.logdebug("distance_reward: %f", distance_reward,
                       logger_name="differential_drive_rl")
        rospy.logdebug("beer_moved_reward: %f", beer_moved_reward,
                       logger_name="differential_drive_rl")

    @staticmethod
    def log_terminate(distance_terminate, time_terminate, beer_moved_terminate):
        if distance_terminate:
            rospy.logdebug("distance_terminate: %f", distance_terminate,
                           logger_name="differential_drive_rl")
        if time_terminate:
            rospy.logdebug("time_terminate: %f", time_terminate,
                           logger_name="differential_drive_rl")
        if beer_moved_terminate:
            rospy.logdebug("beer_moved_terminate: %f",
                           beer_moved_terminate, logger_name="beer_moved_terminate")
