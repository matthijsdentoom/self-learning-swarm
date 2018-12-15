import csv
import datetime
import time
import visualize
import neat
from genome_serializer import GenomeSerializer
from gym_manager import GymScrimmageEnvironment


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
        """This function runs a single experiment and returns its statistics and winner."""

        # Create the population, which is the top-level object for a NEAT run.
        p = neat.Population(self.config)

        # Add a stdout reporter to show progress in the terminal.
        p.add_reporter(neat.StdOutReporter(True))
        stats = neat.StatisticsReporter()
        p.add_reporter(stats)

        # Run for up for the given number of generations
        f = lambda genomes, config: self.eval_genomes(genomes, config, gym_environment=gym_env_manager)
        winner = p.run(f, num_generations)
        # TODO by running one generation in sequence, one might get multiple winners.

        return winner, stats

    def run_and_visualize(self, num_generations, mission_file, output_mission_file, evaluation_steps=2000, index=''):

        print("Running evaluation " + str(index))

        start_time = time.clock()

        gym_env_manager = GymScrimmageEnvironment(mission_file, evaluation_steps)
        self.winner, stats = self.run_experiment(num_generations, gym_env_manager)
        gym_env_manager.close()

        end_time = time.clock()
        run_time = end_time - start_time

        gym_env_manager = GymScrimmageEnvironment(mission_file, evaluation_steps)
        self.winner, stats = self.run_experiment(num_generations, gym_env_manager)
        gym_env_manager.close()

        # Visualise the statistics.
        NeatLearner.visualize_stats(stats, fitness_out_file='avg_fitness' + str(index) + '.svg',
                                    species_out_file='species' + str(index) + '.svg')
        self.output_winner('nn_winner' + str(index), 'winner' + str(index))

        # Write run-time, and current time (used to get back to the winning configuration) to csv file.
        with open('running_stats.csv', 'a') as file:
            writer = csv.writer(file)
            writer.writerow([index, run_time, datetime.datetime.now()])

        # Run the winner in another environment.
        gym_output_env_manager = GymScrimmageEnvironment(output_mission_file, evaluation_steps)
        self.run_genome(self.winner, gym_output_env_manager)
        gym_output_env_manager.close()

    def output_winner(self, net_filename='nn_winner', genome_filename='winner'):
        """This function outputs the current winner in graph and in pickle file."""
        node_names = {-1: '1', -2: '2', -3: '3', -3: '3', -4: '4', -5: '5', -6: '6', -7: '7', 0: 'left', 1: 'right'}
        visualize.draw_net(self.config, self.winner, node_names=node_names, filename=net_filename)

        GenomeSerializer.serialize(self.winner, genome_filename)

        print(self.winner)

    def run_genome(self, genome, gym_environment):
        """This function runs the given genome in the given environment."""

        genome_net = neat.nn.FeedForwardNetwork.create(genome, self.config)
        gym_environment.execute_environment(genome_net)

    @staticmethod
    def visualize_stats(stats, fitness_out_file='avg_fitness.svg', species_out_file='species.svg'):
        """"This function visualizes the output files."""
        visualize.plot_stats(stats, ylog=False, view=False, filename=fitness_out_file)
        visualize.plot_species(stats, view=False, filename=species_out_file)
