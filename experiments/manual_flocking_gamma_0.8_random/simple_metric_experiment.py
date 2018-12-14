import shutil

import sys
sys.path.insert(0, '../../robot_learner')

from os.path import expanduser
from gym_manager import GymScrimmageEnvironment
from neat_learner import NeatLearner

max_evaluation_steps = 2000
num_generations = 1

log_directory = expanduser("~") + "/Documents/scrimmage-logs/"
experiment_directory = log_directory + "flock-experiment"
final_directory = log_directory + "flock-experiment-final"

experiment_mission_file = "zebros_flocking_simple.xml"
output_mission_file = "zebros_flocking_simple_output.xml"


def simple_metric_experiment():

    experiment_manager = GymScrimmageEnvironment(experiment_mission_file, max_evaluation_steps)
    neat_learner = NeatLearner('config-feedforward', experiment_directory)

    neat_learner.run_experiment(num_generations, experiment_manager)

    experiment_manager.close()

    print("Running winner")

    # Print the winner.
    experiment_manager = GymScrimmageEnvironment(output_mission_file, max_evaluation_steps)
    neat_learner.visualize_winner(experiment_manager)

    # Remove the experimental directory, since it is not used anymore.
    shutil.rmtree(experiment_directory)


if __name__ == '__main__':

    simple_metric_experiment()
