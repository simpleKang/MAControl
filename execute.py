# coding=utf-8

import argparse
import time
import logging
from sympy import *

logging.basicConfig(filename='/home/samantha/_train_/logs/execute.log', level=logging.INFO)
logging.info('\n')


def parse_args():
    parser = argparse.ArgumentParser("Rule Executing Experiments for multiagent environments")
    # Environment
    parser.add_argument("--scenario", type=str, default="scenario_DIY", help="name of the scenario script")
    parser.add_argument("--num-episodes", type=int, default=100000, help="number of episodes")
    return parser.parse_args()


def make_env(scenario_name, arglist):
    from multiagent.environment import MultiAgentEnv
    import multiagent.scenarios as scenarios

    # load scenario from script
    scenario = scenarios.load(scenario_name + ".py").Scenario()
    # create world
    world = scenario.make_world()
    # create multiagent environment
    env = MultiAgentEnv(world, scenario.reset_world, scenario.reward, scenario.observation)
    return env


def rule1(obs_n, upl0, downl0, bar, march, num):
    action_n = []
    k = 0
    for obs in obs_n:
        border = get_border(upl0, downl0, bar, march, num, k)
        action = get_action(obs, border, k)
        action_n.append(action)
        k += 1
    print('obs_n', obs_n)
    print('tt', tt)
    print('ss', ss)
    print(action_n)
    return action_n


def rule2(obs_n, upl0, downl0, bar, march, num, L1):
    action_n = []
    global pointA, pointB
    k = 0
    for obs in obs_n:
        border = get_border(upl0, downl0, bar, march, num, k)
        [pointA[k], pointB[k]] = funWP(obs, border, k, L1)
        action = funL1(obs, pointA[k], pointB[k], L1)
        action_n.append(action)
        k += 1
    return action_n


def funWP(obs, border, k, L1):
    pointAk = (-0.9, 0.9)
    pointBk = (0.9, 0.9)
    return [pointAk, pointBk]


def funL1(obs, pointAk, pointBk, L1):
    x, y = symbols('x y')
    [xk, yk] = obs[2:4]
    (xA, yA) = pointAk
    (xB, yB) = pointBk
    out = solve([Eq((xk-x)*(xk-x) + (yk-y)*(yk-y), L1*L1),
                Eq((xA-x)*(yA-yB), (yA-y)*(xA-xB))],
                [x, y])
    action = [0, 0, 0, 0, 0]
    return action


def get_border(upl0, downl0, bar, march, num, k):
    upl = upl0 - bar * k
    downl = downl0 + bar * (num - 1 - k)
    rightlu = (2 * num - 2 * k - 1) * march
    rightld = (2 * k + 1) * march
    return upl, downl, rightlu, rightld

    # action = [0, 0, 0, 1, 0]  # up
    # action = [0, 1, 0, 0, 0]  # right
    # action = [0, 0, 0, 0, 1]  # down


def get_action(obs, border, k):
    upl = border[0]
    downl = border[1]
    rightlu = border[2]
    rightld = border[3]
    action = [0, 1, 0, 0, 0]
    global tt, ss

    if tt[k] == 1:
        ss[k] += 1
        if obs[3] > 0 and ss[k] > rightlu:
            action = [0, 0, 0, 0, 1]
            tt[k] = 0
            print('ops')
        if obs[3] <= 0 and ss[k] > rightld:
            action = [0, 0, 0, 1, 0]
            tt[k] = 0
            print('ops')

    else:
        if obs[1] > 0:
            if obs[3] < upl:
                action = [0, 0, 0, 1, 0]
            else:
                action = [0, 1, 0, 0, 0]
                tt[k] = 1
                ss[k] = 0
                print('come')
        if obs[1] <= 0:
            if obs[3] > downl:
                action = [0, 0, 0, 0, 1]
            else:
                action = [0, 1, 0, 0, 0]
                tt[k] = 1
                ss[k] = 0
                print('come')

    return action


def execute(arglist):

    # Create environment
    env = make_env(arglist.scenario, arglist)
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


if __name__ == '__main__':
    arglist = parse_args()
    execute(arglist)
