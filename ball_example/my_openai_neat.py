import copy
import gym
import scrimmage.utils
import neat

max_evaluation_steps = 200
num_generations = 2


def get_action(nn, observation):
    """Returns the action which the robot does with the current observation"""

    action = nn.activate(observation)[0]
    action = round(action)


    if action > 1: action = 1
    elif action < 0: action = 0

    return action


def execute_environment(net, mission_file_name):

    env = create_scrimmage_env(mission_file_name)

    # the observation is the x position of the vehicle
    # note that a deepcopy is used when a history
    # of observations is desired. This is because
    # the sensor plugin edits the data in-place
    obs = copy.deepcopy(env.reset())

    total_reward = 0
    for i in range(max_evaluation_steps):

        # Note this assumes that the network does not change, ie. feed forward network.
        actions = [[get_action(net, obs[0])], [get_action(net, obs[1])]]
        obs, reward, done = env.step(actions)[:3]
        total_reward += reward

        if done and not i == max_evaluation_steps - 1:
            total_reward = execute_environment(net, env)  # Execute the problem again
            # TODO: prevent this from going in inifinite recursion.
            break

    env.close()

    print("Total Reward: %2.2f" % total_reward)

    return total_reward


def eval_genomes(genomes, config, env):
    """This function evaluates the current genomes in the population."""
    for genome_id, genome in genomes:
        genome.fitness = 0
        net = neat.nn.FeedForwardNetwork.create(genome, config)
        genome.fitness = execute_environment(net, env)


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


def learn_staying_on_line(env, config_path):
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

    # Run for up for the given number of generations
    f = lambda genomes, config: eval_genomes(genomes, config, env=mission_file_name)
    winner = p.run(f, num_generations)


def test_openai(mission_file_name):
    env = create_scrimmage_env(mission_file_name)

    learn_staying_on_line(env, 'config-feedforward')

    env.close()


if __name__ == '__main__':
    mission_file_name = 'openai_mission.xml'

    test_openai(mission_file_name)
