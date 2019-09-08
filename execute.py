# coding=utf-8
import argparse
import time
import MAControl.TESTControl as TESTC
import MAControl.util as U


def parse_args():
    parser = argparse.ArgumentParser("Control Experiments for Multi-Agent Environments")
    parser.add_argument("--scenario", type=str, default="scenario2_Target", help="name of the scenario script")
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

    WorldTarget = []
    for i, landmark in enumerate(world.targets):
        WorldTarget.append([landmark.state.p_pos[0], landmark.state.p_pos[1], landmark.state.p_vel[0],
                            landmark.state.p_vel[1], landmark.value, landmark.defence])
    print('WorldTarget', WorldTarget)
    TESTC.TESTControl.Found_Target_Set = WorldTarget
    TESTC.TESTControl.Found_Target_Info = [[0, 1, 2, 3, 4, 5, 6, 7, 8, 9]]

    # Create Controllers
    Control = []
    for i in range(env.n):
        Control.append(TESTC.TESTControl("agent_%d" % i, env, world, i, arglist))
        Control[i].waypoint_list[Control[i].current_wplist][0:len(U.init_waypoint[i])] = U.init_waypoint[i]

    obs_n = env.reset()
    step = 0
    start = time.time()

    while True:

        # get action
        action_n = []
        for i in range(env.n):
            pointAi, pointBi, finishedi, world = \
                Control[i].PolicyMaker(WorldTarget, obs_n, step, i, world)
            print(pointAi, pointBi, i, finishedi)
            acc_it, acc_il = Control[i].MotionController(obs_n[i], pointAi, pointBi, step)
            actioni = Control[i].InnerController(obs_n[i], acc_it, acc_il, step, finishedi)
            action_n.append(actioni)
        print('Shared_UAV_state: ', TESTC.TESTControl.Shared_UAV_state)
        print('Found_Target_Set: ', TESTC.TESTControl.Found_Target_Set)
        print('Found_Target_Info: ', TESTC.TESTControl.Found_Target_Info)
        print('Target_index: ', TESTC.TESTControl.Target_index)
        print('Resorted_Target: ', TESTC.TESTControl.Resorted_Target)
        print('Selectable_UAV: ', TESTC.TESTControl.Selectable_UAV)
        print('Auctioneer: ', TESTC.TESTControl.Auctioneer)
        print('Trans_step: ', TESTC.TESTControl.Trans_step)
        print('Winner: ', TESTC.TESTControl.Winner)
        print('Price_list: ', TESTC.TESTControl.Price_list)

        # environment step
        new_obs_n, rew_n, done_n, info_n = env.step(action_n)
        step += 1
        obs_n = new_obs_n

        # for displaying
        time.sleep(0.01)
        env.render()
        print('>>>> step', step)
        # print('obs_n', obs_n)
        # print('action_n', action_n)

