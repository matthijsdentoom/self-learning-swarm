import os
import timeit
import pickle
import neat
import sys
sys.path.insert(0, '../../robot_learner')

from gym_manager import GymScrimmageEnvironment
from mission_changer import MissionChanger

num_agents = [3, 5, 8, 10, 15, 30, 50, 80, 100]
evaluation_steps = 2000
num_experiments = 10


def run_time_experiment():

    config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                         neat.DefaultSpeciesSet, neat.DefaultStagnation,
                         'config-feedforward')
    with open("winner0.pickle", 'rb') as pickle_file:

        genome = pickle.load(pickle_file)

    net = neat.nn.FeedForwardNetwork.create(genome, config)

    missionChanger = MissionChanger('zebros_flocking_simple.xml')
    missionChanger.enable_multi_threading(4)

    run_times = []


    try:

        for num in num_agents:

            mission_filename = 'zebros_flocking_simple_{}.xml'.format(num)
            missionChanger.update_entity_count('RobotLearner', num)
            missionChanger.write(mission_filename)

            gym_env_manager = GymScrimmageEnvironment(mission_filename, evaluation_steps)
            average_runtime = timeit.timeit(lambda: gym_env_manager.execute_environment(net), number=10)

            result = str(num) + "\t" + str(average_runtime)
            print(result)

            run_times.append(result)

            gym_env_manager.close()

            os.remove(mission_filename)

    except Exception:
        print("Error, but making sure results are printed")

    for result in run_times:
        print(result)


if __name__ == '__main__':

    run_time_experiment()
