#!/usr/bin/env python
import rospy
from reinforcement_learning_steering.msg import RLSimulation
from gazebo_msgs.msg import ModelStates
import csv
import os


def rl_simulation_logger(data):
    csv_file = 'logger.csv'
    if data.terminate:
        with open(csv_file, 'a') as csvfile:
            csv_writer = csv.writer(csvfile, delimiter=',')
            if not os.path.exists(csv_file):
                print 'writing header to file'
                csv_writer.writerow(['episode_iteration', 'total_reward'])
            print '{}, {}'.format(data.episode_iteration, data.current_episode.total_reward)
            csv_writer.writerow([data.episode_iteration, data.current_episode.total_reward])


if __name__ == "__main__":
    rospy.init_node('rl_diff_logger')

    rospy.Subscriber('/differential_drive_model/rl', RLSimulation, rl_simulation_logger, queue_size=1)

    rospy.spin()
