# coding=utf-8

import argparse
import numpy as np
import time
import os
import MAControl.Util.OnlineCoverRate as CR
import MAControl.Test_Auction.InnerController_PID as IC_P
import MAControl.Test_Auction.MotionController_L1_TECS as MC_L
import MAControl.Test_Auction.PathPlanner_Simple as PP_S
import MAControl.Test_Auction.PathPlanner_generate_at_present as PP_G
import MAControl.Test_Auction.PolicyMaker_Auction as PM_A
import MAControl.Test_Auction.PolicyMaker_Weight as PM_V
import MAControl.Test_Auction.PolicyMaker_Weight_T as PM_T
import MAControl.Test_Movable_Target_Policy.InnerController_PID as T_IC_P
import MAControl.Test_Movable_Target_Policy.MotionController_L1_TECS as T_MC_L
import MAControl.Test_Movable_Target_Policy.PathPlanner_Simple as T_PP_S
import MAControl.Test_Movable_Target_Policy.PolicyMaker_Auction as T_PM_A


def parse_args():
    parser = argparse.ArgumentParser("Control Experiments for Multi-Agent Environments")
    parser.add_argument("--scenario", type=str, default="scenario4_Xuxiao", help="name of the scenario script")
    parser.add_argument("--step_max", type=int, default=4000, help="maximum steps")
    parser.add_argument("--cover_edge", type=int, default=200, help="number of cells of one edge")
    parser.add_argument("--OnlineCoverRate", type=int, default=0, help="Online or offline to calculate cover rate")
    return parser.parse_args()


def make_env(arglist):
    from MAEnv.environment import MultiAgentEnv
    import MAEnv.scenarios as scenarios

    # load scenario from script
    scenario = scenarios.load(arglist.scenario + ".py").Scenario()

    # create world and env
    world = scenario.make_world()
    env = MultiAgentEnv(world, scenario.reset_world, scenario.reward, scenario.observation)

    # creat WorldTarget
    WorldTarget = list()
    for i, landmark in enumerate(world.targets):
        WorldTarget.append([landmark.state.p_pos[0], landmark.state.p_pos[1], landmark.value, landmark.defence])

    return env, world, WorldTarget


def get_controller(env, world, arglist):

    ControllerSet = list()

    # 初始化小瓜子
    for i in range(env.n - len(world.movable_targets)):
        control = list()
        # control.append(PM_A.PolicyMaker_Auction("agent_%d" % i, env, world, i, arglist))
        # control.append(PM_V.PolicyMaker_Weight("agent_%d" % i, env, world, i, arglist))
        control.append(PM_T.PolicyMaker_Weight_T("agent_%d" % i, env, world, i, arglist))

        # control.append(PP_S.PathPlanner_Simple("agent_%d" % i, env, world, i, arglist))
        control.append(PP_G.PathPlanner_generate_at_present("agent_%d" % i, env, world, i, arglist))

        control.append(MC_L.MotionController_L1_TECS("agent_%d" % i, env, world, i, arglist))
        control.append(IC_P.InnerController_PID("agent_%d" % i, env, world, i, arglist))
        control.append(False)  # Arriveflag
        control.append(False)  # Isattacking
        ControllerSet.append(control)

    # 初始化动目标
    for i in range(len(world.movable_targets)):
        control = list()
        control.append(T_PM_A.PolicyMaker_Target("movable_target_%d" % i, env, world, i, arglist))
        control.append(T_PP_S.PathPlanner_Simple("movable_target_%d" % i, env, world, i, arglist))
        control.append(T_MC_L.MotionController_L1_TECS("movable_target_%d" % i, env, world, i, arglist))
        control.append(T_IC_P.InnerController_PID("movable_target_%d" % i, env, world, i, arglist))
        control.append(False)  # Arriveflag
        control.append(False)  # Isattacking
        ControllerSet.append(control)

    return ControllerSet


def update_action(env, world, WorldTarget, obs_n, step, NewController):

    # get action
    action_n = list()

    # 小瓜子运动
    for i in range(env.n - len(world.movable_targets)):

        list_i = NewController[i][0]. \
            make_policy(WorldTarget, obs_n, step)

        pointAi, pointBi, finishedi, NewController[i][5], WorldTarget = NewController[i][1].\
            planpath(list_i, obs_n[i], NewController[i][4], step, WorldTarget)

        acctEi, acclEi, NewController[i][4] = NewController[i][2]. \
            get_expected_action(obs_n[i], pointAi, pointBi, step, finishedi)

        actioni = NewController[i][3]. \
            get_action(obs_n[i], acctEi, acclEi, step, finishedi)

        action_n.append(actioni)

    # 动目标运动
    for i in range(env.n - len(world.movable_targets), env.n):

        list_i = NewController[i][0]. \
            make_policy(WorldTarget, obs_n, step)

        pointAi, pointBi, finishedi, NewController[i][5] = NewController[i][1]. \
            planpath(list_i, obs_n[i], NewController[i][4], step)

        acctEi, acclEi, NewController[i][4] = NewController[i][2]. \
            get_expected_action(obs_n[i], pointAi, pointBi, step, finishedi)

        actioni = NewController[i][3]. \
            get_action(obs_n[i], acctEi, acclEi, step, finishedi)

        action_n.append(actioni)

    return action_n, WorldTarget


def augment_view(env, world, NewController):
    for i in range(env.n):
        if NewController[i][5]:
            world.agents[i].attacking = True


if __name__ == '__main__':

    arglist = parse_args()

    # Create environment
    env, world, worldtarget = make_env(arglist)

    with open(os.path.dirname(__file__) + '/track/para.txt', 'w') as f:
        f.write(str(arglist.cover_edge)+'\n')
        f.write(str(env.n - len(world.movable_targets)) + '\n')

    # Create Controller
    NewController = get_controller(env, world, arglist)

    obs_n = env.reset()
    step = 0
    start = time.time()

    # 为每个小瓜子创建路径文件
    for k in range(env.n - len(world.movable_targets)):
        open(os.path.dirname(__file__) + '/track/agent_%d_track.txt' % k, 'w')

    # if arglist.OnlineCoverRate:
    #     last_cover = list()
    #     area = np.zeros((arglist.cover_edge, arglist.cover_edge))
    #     open('cover_rate.txt', 'w')
    #     for k in range(env.n - len(world.movable_targets)):
    #         last_cover.append([])

    while True:

        # get action
        # print('>>>> step', step)
        action_n, worldtarget = update_action(env, world, worldtarget, obs_n, step, NewController)

        # environment step
        new_obs_n, rew_n, done_n, info_n = env.step(action_n)
        step += 1
        obs_n = new_obs_n

        # if arglist.OnlineCoverRate:
        #     if step % 5 == 0:
        #         area, last_cover = CR.update_area_cover(arglist.cover_edge, area, last_cover, obs_n,
        #                                                 env.n - len(world.movable_targets))
        #         cover_rate, overlap_rate = CR.cal_cover_rate(area)
        #         with open('cover_rate.txt', 'a') as c:
        #             c.write(str(step)+' '+str(cover_rate)+' '+str(overlap_rate)+'\n')
        #         print('>>>> cover rate ', cover_rate)

        # 记录每个小瓜子每个step的位置
        for k in range(env.n - len(world.movable_targets)):
            with open(os.path.dirname(__file__) + '/track/agent_%d_track.txt' % k, 'a') as f:
                f.write(str(obs_n[k][0])+' '+str(obs_n[k][1])+' '+str(obs_n[k][2])+' '+str(obs_n[k][3])+'\n')

        # for displaying
        # time.sleep(0.01)
        augment_view(env, world, NewController)
        env.render()
        print('>>>> step', step)
