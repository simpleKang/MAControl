# coding=utf-8

import argparse
import time

import MAControl.Default1.InnerController_PID as IC_P
import MAControl.Default1.MotionController_L1_TECS as MC_L
import MAControl.Default1.PathPlanner_EgdeWaypoint as PP_G
import MAControl.Default1.PolicyMaker_LocalDecision as PM_L


def parse_args():

    parser = argparse.ArgumentParser("Control Experiments for Multi-Agent Environments")

    # Environment
    parser.add_argument("--scenario", type=str, default="scenario6_AFIT", help="name of the scenario script")
    parser.add_argument("--uav-num", type=int, default=10, help="number of uav")
    parser.add_argument("--train-step-max", type=int, default=10000, help="number of episodes")
    parser.add_argument("--episode-step-max", type=int, default=4000, help="maximum episode length")
    parser.add_argument("--display-step-max", type=int, default=4000, help="number of episodes for displaying")
    parser.add_argument("--learn-num", type=int, default=50, help="number of learning rounds after collecting data")
    parser.add_argument("--data-collect-num", type=int, default=1, help="number of data collector")

    # Evaluation
    parser.add_argument("--restore", action="store_true", default=False)
    parser.add_argument("--display", action="store_true", default=True)

    return parser.parse_args()


def make_env(arglist):
    from MAEnv.environment import MultiAgentEnv
    import MAEnv.scenarios as scenarios

    # load scenario from script
    scenario = scenarios.load(arglist.scenario + ".py").Scenario()

    # create world and env
    world_ = scenario.make_World(arglist.uav_num)
    env_ = MultiAgentEnv(world_, scenario.reset_World, scenario.reward, scenario.observation)

    # creat WorldTarget
    worldtarget_ = list()
    for i, landmark in enumerate(world_.targets):
        worldtarget_.append([landmark.state.p_pos[0], landmark.state.p_pos[1], landmark.value, landmark.defence])

    return env_, world_, worldtarget_


def get_controller(env, world, arglist):

    ControllerSet = list()

    # 初始化小瓜子
    for i in range(arglist.agent_num):
        control = list()

        control.append(PM_L.PolicyMaker_LocalDecision("agent_%d" % i, env, world, i, arglist))
        control.append(PP_G.PathPlanner_EgdeWaypoint("agent_%d" % i, env, world, i, arglist))
        control.append(MC_L.MotionController_L1_TECS("agent_%d" % i, env, world, i, arglist))
        control.append(IC_P.InnerController_PID("agent_%d" % i, env, world, i, arglist))
        control.append(False)  # Arriveflag
        control.append(False)  # Isattacking

        ControllerSet.append(control)

    return ControllerSet


def action(arglist, WorldTarget, obs_n, step, NewController):

    # get action
    action_n = list()

    # 小瓜子运动
    for i in range(arglist.agent_num):

        list_i, sight_friends = NewController[i][0]. \
            make_policy(WorldTarget, obs_n, step)

        pointAi, pointBi, finishedi, NewController[i][5], WorldTarget = NewController[i][1].\
            planpath(list_i, obs_n[i], NewController[i][4], step, WorldTarget)

        acctEi, acclEi, NewController[i][4] = NewController[i][2]. \
            get_expected_action(obs_n[i], pointAi, pointBi, step, finishedi)

        actioni = NewController[i][3]. \
            get_action(obs_n[i], acctEi, acclEi, step, finishedi)

        action_n.append(actioni)

    return action_n, WorldTarget


def augment_view(arglist, world, NewController):
    for i in range(arglist.agent_num):
        if NewController[i][5]:
            world.agents[i].attacking = True


if __name__ == '__main__':

    arglist = parse_args()

    # Create environment
    env, world, worldtarget = make_env(arglist)
    # 手动输入状态维度 n_features

    # Create Controller
    NewController = get_controller(env, world, arglist)

    if arglist.display:

        for num in range(arglist.data_collect_num):

            obs_n = env.reset()
            start = time.time()

            for step in range(arglist.display_step_max):

                # 选择动作
                action_n, worldtarget = action(arglist, worldtarget, obs_n, step, NewController)

                new_obs_n, rew_n, done_n, info_n = env.step(action_n)

                obs_n = new_obs_n

                # 画图展示
                augment_view(arglist, world, NewController)
                env.render()
                print('>>> Num', num, '>>>> step', step)
                time.sleep(0.01)

            time.sleep(1)
            end = time.time()
            interval = round((end - start), 2)
            print('Time Interval ', interval)
