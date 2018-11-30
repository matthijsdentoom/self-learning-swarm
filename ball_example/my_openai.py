import copy
from time import sleep

import gym
import scrimmage.utils
import neat
import random

simulation_steps = 200
num_generations = 150

def get_action(obs):
    return random.randint(0, 1)


def execute_environment(mission_file_name):

    env = create_scrimmage_env(mission_file_name)

    # the observation is the x position of the vehicle
    # note that a deepcopy is used when a history
    # of observations is desired. This is because
    # the sensor plugin edits the data in-place
    obs = []
    temp_obs = copy.deepcopy(env.reset())
    obs.append(temp_obs)
    total_reward = 0
    for i in range(simulation_steps):

        print(temp_obs)
        actions = [[get_action(obs)], [get_action(obs)]]
        temp_obs, reward, done = env.step(actions)[:3]
        obs.append(copy.deepcopy(temp_obs))
        total_reward += reward

        # print("reward: " + str(reward))
        # print(temp_obs)

        if done and not i == simulation_steps - 1:
            total_reward = execute_environment(mission_file_name)  # Execute the problem again
            break

    env.close()

    print("Total Reward: %2.2f" % total_reward)

    return total_reward


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


def learn_staying_on_line(config_path):

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
    # p.add_reporter(neat.Checkpointer(5))


if __name__ == '__main__':

    mission_file_name = 'openai_mission.xml'

    # env = create_scrimmage_env(mission_file_name)

    for i in range(0, 2000):
        execute_environment(mission_file_name)
