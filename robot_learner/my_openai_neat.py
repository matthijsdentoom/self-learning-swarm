import copy
import shutil

import gym
import scrimmage.utils
import neat
import csv
import visualize
from os.path import expanduser

max_evaluation_steps = 2000
num_generations = 150

log_directory = expanduser("~") + "/Documents/scrimmage-logs/"
experiment_directory = log_directory + "/flock-experiment"
num_robots = 10


def get_action(nn, observation):
    """Returns the action which the robot does with the current observation"""

    actions = nn.activate(observation)

    return actions


def execute_environment(net, mission_file_name):

    env = create_scrimmage_env(mission_file_name)

    # the observation is the x position of the vehicle
    # note that a deepcopy is used when a history
    # of observations is desired. This is because
    # the sensor plugin edits the data in-place
    obs = copy.deepcopy(env.reset())

    for i in range(max_evaluation_steps):

        # Note this assumes that the network does not change, ie. feed forward network.
        actions = [[get_action(net, obs[i][:-1])] for i in range(num_robots)]
        obs, reward, done = env.step(actions)[:3]

        if done and not i == max_evaluation_steps - 1:
            total_reward = execute_environment(net, mission_file_name)  # Execute the problem again
            # TODO: prevent this from going in infinite recursion.

            return total_reward

    env.close()

    total_reward = 0

    # TODO, only works single-threaded, because using latest
    with open(experiment_directory + '/latest/summary.csv', mode='r') as csv_file:
        csv_reader = csv.DictReader(csv_file)
        for row in csv_reader:
            # Assume only one row is there with the score.
            total_reward = float(row['score'])

    return total_reward


def eval_genomes(genomes, config, mission_file_name):
    """This function evaluates the current genomes in the population."""
    for genome_id, genome in genomes:
        genome.fitness = 0
        net = neat.nn.FeedForwardNetwork.create(genome, config)
        genome.fitness = execute_environment(net, mission_file_name)


def remove_scrimmage_env():

    if 'scrimmage-v0' in gym.envs.registry.env_specs.keys():
        del gym.envs.registry.env_specs['scrimmage-v0']


def create_scrimmage_env(mission_file_name):
    """Create the scrimmage environment of the specified mission file."""

    try:
        env = gym.make('scrimmage-v0')

    except gym.error.Error:
        mission_file = scrimmage.utils.find_mission(mission_file_name)

        gym.envs.register(
            id='scrimmage-v0',
            entry_point='scrimmage.bindings:ScrimmageOpenAIEnv',
            max_episode_steps=1e9,
            reward_threshold=1e9,
            kwargs={
                "enable_gui": False,
                "mission_file": mission_file}
        )
        env = gym.make('scrimmage-v0')

    return env


def learn_flocking(mission_file_name, config_path):
    # Load configuration.
    config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                         neat.DefaultSpeciesSet, neat.DefaultStagnation,
                         config_path)

    # Create the population, which is the top-level object for a NEAT run.
    p = neat.Population(config)

    # Add a stdout reporter to show progress in the terminal.
    p.add_reporter(neat.StdOutReporter(True))
    stats = neat.StatisticsReporter()
    p.add_reporter(stats)

    # Run for up for the given number of generations
    f = lambda genomes, config: eval_genomes(genomes, config, mission_file_name=mission_file_name)
    winner = p.run(f, num_generations)

    winner_net = neat.nn.FeedForwardNetwork.create(winner, config)

    remove_scrimmage_env()
    execute_environment(winner_net, "zebros_flocking_output.xml")

    node_names = {-1:'1', -2: '2', -3:'3', -3: '3', -4:'4', -5: '5', -6:'6', -7: '7', 0:'left', 1:'right'}
    visualize.draw_net(config, winner, node_names=node_names)
    visualize.plot_stats(stats, ylog=False, view=False)
    visualize.plot_species(stats, view=False)

    print(winner)

    # Remove the experimental directory, since it is not used anymore.
    shutil.rmtree(experiment_directory)


def main():

    mission_file_name = 'zebros_flocking.xml'

    learn_flocking(mission_file_name, 'config-feedforward')


if __name__ == '__main__':

    main()
