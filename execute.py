# coding=utf-8
import argparse
import time
import MAControl.TESTControl as TESTC
import MAControl.util as U
import MAControl.Test_Auction.InnerController_PID as IC_P
import MAControl.Test_Auction.MotionController_L1_TECS as MC_L
import MAControl.Test_Auction.PathPlanner_Simple as PP_S
import MAControl.Test_Auction.PolicyMaker_Auction as PM_A


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

    # >>>>>> TEST NEW CONTROLLERS >>>>>>>
    NewController = []
    for i in range(env.n):
        control = []
        control.append(PM_A.PolicyMaker_Auciton("agent_%d" % i, env, world, i, arglist))
        control.append(PP_S.PathPlanner_Simple("agent_%d" % i, env, world, i, arglist))
        control.append(MC_L.MotionController_L1_TECS)
        control.append(IC_P.InnerController_PID)
        NewController.append(control)

    obs_n = env.reset()
    step = 0
    start = time.time()

    WorldTarget = []
    for i, landmark in enumerate(world.targets):
        WorldTarget.append([landmark.state.p_pos[0], landmark.state.p_pos[1], landmark.state.p_vel[0],
                            landmark.state.p_vel[1], landmark.value, landmark.defence])
    print('WorldTarget', WorldTarget)
    # TESTC.TESTControl.Found_Target_Set = WorldTarget
    # TESTC.TESTControl.Found_Target_Info = [[0, 1, 2, 3, 4, 5, 6, 7, 8, 9], [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]]

    while True:

        # get action
        action_n = []
        for i in range(env.n):
            NewController[i][0].makepolicy(WorldTarget, obs_n, step)
            NewController[i][1].planpath(2)
            NewController[i][2].controlmotion(3)
            NewController[i][3].controlinner(4)






            # pointAi, pointBi, finishedi, world = \
            #     Control[i].PolicyMaker(WorldTarget, obs_n, step, i, world)
            # print(pointAi, pointBi, i, finishedi)
            # acc_it, acc_il = Control[i].MotionController(obs_n[i], pointAi, pointBi, step)
            # actioni = Control[i].InnerController(obs_n[i], acc_it, acc_il, step, finishedi)
            # action_n.append(actioni)


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

