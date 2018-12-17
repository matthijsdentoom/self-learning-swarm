import sys
sys.path.insert(0, '../../robot_learner')

import timeit
import pickle
import neat
from gym_manager import GymScrimmageEnvironment


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

    run_times = []

    try:

        for num in num_agents:

            gym_env_manager = GymScrimmageEnvironment('run_time/zebros_flocking_simple_{}.xml'.format(num), evaluation_steps)
            average_runtime = timeit.timeit(lambda: gym_env_manager.execute_environment(net), number=10)

            result = str(num) + "\t" + str(average_runtime)
            print(result)

            run_times.append(result)

            gym_env_manager.close()

    except Exception:
        print("Error, but making sure results are printed")

    for result in run_times:
        print(result)


if __name__ == '__main__':

    run_time_experiment()
