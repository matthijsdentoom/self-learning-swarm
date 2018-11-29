import gym
import neat
import os

max_evaluation_steps    = 100000
num_generations         = 150

def eval_genomes(genomes, config, env):
    for genome_id, genome in genomes:

        genome.fitness = 0
        net = neat.nn.FeedForwardNetwork.create(genome, config)
        genome.fitness = eval_network(net, env)


def eval_network(net, env, render=False):
    """Evaluate the given neural network in the given environment."""

    env.reset()

    # First do nothing to get an observation.
    observation = []

    summed_reward = 0
    for t in range(max_evaluation_steps):

        if render:      # Render if that is being asked.
            env.render()

        if len(observation) == 0:
            action = env.action_space.sample()
        else:
            action = round(net.activate(observation)[0])

        observation, reward, done, info = env.step(action)
        summed_reward += reward

        if done:    # If the environment has failed, by turning over or by getting out of the field, return reward.
            if render: print(info)

            break

    return summed_reward


def learn_cartpole(config_path):

    env = gym.make('CartPole-v0')
    env._max_episode_steps =  max_evaluation_steps

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
    p.add_reporter(neat.Checkpointer(5))


    # Run for up for the given number of generations
    f = lambda genomes, config: eval_genomes(genomes, config, env=env)
    winner = p.run(f, num_generations)

    input("Press Enter to continue...")

    net = neat.nn.FeedForwardNetwork.create(winner, config)
    eval_network(net, env, True)


if __name__ == '__main__':
    local_dir = os.path.dirname(__file__)
    config_path = os.path.join(local_dir, 'config-feedforward')

    learn_cartpole(config_path)
