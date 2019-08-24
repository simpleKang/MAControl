# coding=utf-8
import argparse
import time
import MAControl.TESTControl as TESTC


def parse_args():
    parser = argparse.ArgumentParser("Control Experiments for Multi-Agent Environments")
    parser.add_argument("--scenario", type=str, default="scenario_DIY", help="name of the scenario script")
    parser.add_argument("--step-max", type=int, default=4000, help="maximum steps")
    return parser.parse_args()


def make_env(arglist):
    print('make_env')
    from MAEnv.environment import MultiAgentEnv
    import MAEnv.scenarios as scenarios

    # load scenario from script
    scenario = scenarios.load(arglist.scenario + ".py").Scenario()

    # create world and env
    world = scenario.make_world()
    env = MultiAgentEnv(world, scenario.reset_world, scenario.reward, scenario.observation)
    return env, world


if __name__ == '__main__':
    arglist = parse_args()

    # Create environment
    env, world = make_env(arglist)

    # Create Controllers
    Control = []
    for i in range(env.n):
        Control.append(TESTC.TESTControl("agent_%d" % i, env, world, i, arglist))

    obs_n = env.reset()
    start = time.time()

    for step in range(arglist.step_max):
        for agent_index in range(env.n):
            Control.

            pointA_n, PointB_n = TC.PathPlanner(obs_n)
            action_n = TC.MotionController(pointA_n, pointB_n)

    while True:

        # get action
        action_n = rule1(obs_n, 0.9, -0.9, 0.2, 4, env.n)

        # environment step
        new_obs_n, rew_n, done_n, info_n = env.step(action_n)
        step += 1
        done = all(done_n)
        obs_n = new_obs_n

        # for displaying
        time.sleep(0.05)
        env.render()
