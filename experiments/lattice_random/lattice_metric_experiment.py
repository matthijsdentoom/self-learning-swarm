import shutil

import sys
sys.path.insert(0, '../../robot_learner')

from os.path import expanduser
from neat_learner import NeatLearner

max_evaluation_steps = 2000
num_generations = 100

log_directory = expanduser("~") + "/Documents/scrimmage-logs/"
experiment_directory = log_directory + "flock-experiment"
final_directory = log_directory + "flock-experiment-final"

experiment_mission_file = "zebros_flocking_lattice.xml"
output_mission_file = "zebros_flocking_lattice_output.xml"


def lattice_metric_experiment_func():

    neat_learner = NeatLearner('config-feedforward', experiment_directory)

    for i in range(5):
        neat_learner.run_and_visualize(num_generations, experiment_mission_file, output_mission_file, index=i)

        # Remove the experimental directory, since it is not used anymore.
        shutil.rmtree(experiment_directory)


if __name__ == '__main__':

    lattice_metric_experiment_func()
