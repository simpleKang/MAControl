# coding=utf-8

# import numpy as np
# import controls
import argparse
import time
import logging

logging.basicConfig(filename='/home/samantha/_train_/logs/execute.log', level=logging.INFO)
logging.info('\n')


def parse_args():
    parser = argparse.ArgumentParser("Rule Executing Experiments for multiagent environments")
    # Environment
    parser.add_argument("--scenario", type=str, default="scenario_DIY", help="name of the scenario script")
    parser.add_argument("--num-episodes", type=int, default=100000, help="number of episodes")
    return parser.parse_args()


def make_env(arglist):
    from MAEnv.environment import MultiAgentEnv
    import MAEnv.scenarios as scenarios

    # load scenario from script
    scenario = scenarios.load(arglist.scenario + ".py").Scenario()  # 建立一个类（？）
    # create world
    world = scenario.make_world()
    # create multiagent environment
    env = MultiAgentEnv(world, scenario.reset_world, scenario.reward, scenario.observation)
    return env, world


if __name__ == '__main__':
    arglist = parse_args()

    # Create environment
    env, world = make_env(arglist)
    obs_n = env.reset()
    episode_step = 0

    # Start iteration
    print('Starting iterations...')
    logging.info('Starting iterations...')
    global tt, ss, pointA, pointB
    tt = [0 for i in range(env.n)]
    ss = [0 for i in range(env.n)]
    pointA = [(0, 0) for i in range(env.n)]
    pointB = [(0, 0) for i in range(env.n)]

    while True:

        # get action
        action_n = rule1(obs_n, 0.9, -0.9, 0.2, 4, env.n)

        # environment step
        new_obs_n, rew_n, done_n, info_n = env.step(action_n)
        episode_step += 1
        done = all(done_n)
        obs_n = new_obs_n

        # todo # 所有agent离开屏幕后终止本次试验

        if done:
            obs_n = env.reset()
            episode_step = 0

        # for displaying
        time.sleep(0.05)
        env.render()
