import csv
import visualize
import neat


class NeatLearner:
    """"
    This class is used to learn behaviours to the robots.
    The goal of this class is to make a generic class, which can be used for multiple experiments.
    """

    def __init__(self, neat_config_file, experiment_directory):

        self.experiment_directory = experiment_directory
        self.winner = None
        self.config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                                  neat.DefaultSpeciesSet, neat.DefaultStagnation,
                                  neat_config_file)

    def evaluate(self, net, gym_environment):
        """This class evaluates the given network and executes it in the environment."""

        gym_environment.execute_environment(net)

        total_reward = 0
        # TODO, only works single-threaded, because using latest
        with open(self.experiment_directory + '/latest/summary.csv', mode='r') as csv_file:
            csv_reader = csv.DictReader(csv_file)
            for row in csv_reader:
                # Assume only one row is there with the score.
                total_reward = float(row['score'])

        return total_reward

    def eval_genomes(self, genomes, config, gym_environment):
        """This function evaluates the current genomes in the population."""
        for genome_id, genome in genomes:
            genome.fitness = 0
            net = neat.nn.FeedForwardNetwork.create(genome, config)
            genome.fitness = self.evaluate(net, gym_environment)

    def run_experiment(self, num_generations, gym_env_manager):

        # Create the population, which is the top-level object for a NEAT run.
        p = neat.Population(self.config)

        # Add a stdout reporter to show progress in the terminal.
        p.add_reporter(neat.StdOutReporter(True))
        stats = neat.StatisticsReporter()
        p.add_reporter(stats)

        # Run for up for the given number of generations
        f = lambda genomes, config: self.eval_genomes(genomes, config, gym_environment=gym_env_manager)
        self.winner = p.run(f, num_generations)
        # TODO by running one generation in sequence, one might get multiple winners.

        visualize.plot_stats(stats, ylog=False, view=False)
        visualize.plot_species(stats, view=False)

    def visualize_winner(self, gym_environment):

        winner_net = neat.nn.FeedForwardNetwork.create(self.winner, self.config)

        gym_environment.execute_environment(winner_net)

        node_names = {-1: '1', -2: '2', -3: '3', -3: '3', -4: '4', -5: '5', -6: '6', -7: '7', 0: 'left', 1: 'right'}
        visualize.draw_net(self.config, self.winner, node_names=node_names)

        print(self.winner)

        with open("winning_genome.txt", "w") as text_file:
            text_file.write(str(self.winner))
