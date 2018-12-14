import math

import gym
import scrimmage.utils
import copy
from helper_functions import clip

min_range = 2
min_consider = 4
max_consider = 5
max_range = 7
comm_range = 10

max_speed = 5


def get_action(skirt, closest_heading):

    direction = 0
    speed = 1

    for i in range(len(skirt)):

        sensor_angle = 2 * math.pi * (i / len(skirt)) + math.pi / len(skirt)

        if i >= len(skirt / 2):
            sensor_angle = -sensor_angle + math.pi

        if skirt[i] < min_range:
            direction += math.sin(sensor_angle)
            speed = min(speed, skirt[i] / min_consider)
        elif skirt[i] < min_consider:
            factor = 1 - ((skirt[i] - min_range) / (min_consider - min_range))
            direction += math.sin(sensor_angle) * factor
            speed = min(speed, skirt[i] / min_consider)

        elif skirt[i] >= comm_range:
            continue
        elif skirt[i] > max_range:
            direction -= math.sin(sensor_angle)
        elif skirt[i] > max_consider:
            attraction_interval = max_range - max_consider
            direction -= math.sin(sensor_angle) * ((skirt[i] - max_consider) / attraction_interval)

    # Take the decision into account.
    if -0.5 < direction < 0.5:
        closest_heading = clip(-0.5 * math.pi, 0.5 * math.pi, closest_heading)
        direction_update = math.sin(closest_heading)
        direction = (direction + direction_update) / 2

    # limit the direction.
    direction = clip(-1, 1, direction)
    speed = clip(0, 1, speed)

    # Calculate motor speeds.
    left = (speed + direction) * max_speed / 2
    right = (speed - direction) * max_speed / 2

    left = clip(-max_speed, max_speed, left)
    right = clip(-max_speed, max_speed, right)

    return [left, right]


if __name__ == '__main__':
    mission_file_name = 'zebros_flocking_simple.xml'
    max_evaluation_steps = 10000

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
                "enable_gui": True,
                "mission_file": mission_file}
        )
        env = gym.make('scrimmage-v0')

    # the observation is the x position of the vehicle
    # note that a deepcopy is used when a history
    # of observations is desired. This is because
    # the sensor plugin edits the data in-place
    obs = copy.deepcopy(env.reset())

    for i in range(max_evaluation_steps):

        # Note this assumes that the network does not change, ie. feed forward network.
        actions = [[get_action(obs[i][:-1], obs[i][-1])] for i in range(len(obs))]
        obs, reward, done = env.step(actions)[:3]

    env.close()
