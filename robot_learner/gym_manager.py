import gym
import scrimmage.utils
import copy


class GymScrimmageEnvironment:
    """"This class manages the gym environment and executes experiments."""

    def __init__(self, mission_file, evaluation_steps, gym_env_name='scrimmage-v0'):
        self.mission_file = mission_file
        self.evaluation_steps = evaluation_steps
        self.gym_env_name = gym_env_name

    def execute_environment(self, net):

        env = self.create_scrimmage_env()

        # the observation is the x position of the vehicle
        # note that a deepcopy is used when a history
        # of observations is desired. This is because
        # the sensor plugin edits the data in-place
        obs = copy.deepcopy(env.reset())

        for i in range(self.evaluation_steps):

            # Note this assumes that the network does not change, ie. feed forward network.
            actions = [[self.get_action(net, obs[i][:-1])] for i in range(len(obs))]
            obs, reward, done = env.step(actions)[:3]

            if done and not i == self.evaluation_steps - 1:
                total_reward = self.execute_environment(net)  # Execute the problem again
                # TODO: prevent this from going in infinite recursion.

                return total_reward

        env.close()

    @staticmethod
    def get_action(nn, observation):
        """Returns the action which the robot does with the current observation"""
        actions = nn.activate(observation)

        return actions

    def create_scrimmage_env(self):
        """Create the scrimmage environment of the specified mission file."""

        try:
            env = gym.make(self.gym_env_name)

        except gym.error.Error:
            mission_file = scrimmage.utils.find_mission(self.mission_file)

            gym.envs.register(
                id='scrimmage-v0',
                entry_point='scrimmage.bindings:ScrimmageOpenAIEnv',
                max_episode_steps=1e9,
                reward_threshold=1e9,
                kwargs={
                    "enable_gui": False,
                    "mission_file": mission_file}
            )

            env = gym.make(self.gym_env_name)

        return env

    def close(self):
        """This function closes the open gym scrimmage environment and should be called before another environment
            is opened."""
        if self.gym_env_name in gym.envs.registry.env_specs.keys():
            del gym.envs.registry.env_specs[self.gym_env_name]
