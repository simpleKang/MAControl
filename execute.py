# coding=utf-8

import argparse
import numpy as np
import time
import os
import MAControl.Util.OfflineCoverRate as OCR

import MAControl.Default.InnerController_PID as IC_P
import MAControl.Default.MotionController_L1_TECS as MC_L
import MAControl.Default.PathPlanner_EgdeWaypoint as PP_G
import MAControl.Default.PolicyMaker_SelfOrganization as PM_S


def parse_args():

    parser = argparse.ArgumentParser("Control Experiments for Multi-Agent Environments")

    # Environment
    parser.add_argument("--scenario", type=str, default="scenario6_AFIT", help="name of the scenario script")
    parser.add_argument("--uav-num", type=int, default=10, help="number of uav")
    parser.add_argument("--display-step-max", type=int, default=1000, help="number of episodes for displaying")
    parser.add_argument("--data-collect-num", type=int, default=3, help="number of data collector")
    parser.add_argument("--cover-edge", type=int, default=200, help="number of cells of one edge")

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
    for i, agent in enumerate(world_.T_agents):
        worldtarget_.append([agent.state.p_pos[0], agent.state.p_pos[1], agent.w, agent.H])

    return env_, world_, worldtarget_


def get_controller(env, world, arglist):

    ControllerSet = list()

    # 初始化小瓜子
    for i in range(arglist.uav_num):
        control = list()

        control.append(PM_S.PolicyMaker_SelfOrganization("agent_%d" % i, env, world, i, arglist))
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
    for i in range(arglist.uav_num):

        list_i = NewController[i][0].\
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
    for i in range(arglist.uav_num):
        if NewController[i][5]:
            world.agents[i].attacking = True


if __name__ == '__main__':

    arglist = parse_args()

    # Create environment
    env, world, worldtarget = make_env(arglist)
    # 手动输入状态维度 n_features

    # Create Controller
    NewController = get_controller(env, world, arglist)

    for num in range(arglist.data_collect_num):

        with open(os.path.dirname(__file__) + '/track/para.txt', 'w') as f:
            f.write(str(arglist.cover_edge)+' '+str(arglist.agent_num)+' '+str(arglist.display_step_max))

        # 为每个小瓜子创建状态文件
        for k in range(arglist.agent_num):
            open(os.path.dirname(__file__) + '/track/agent_%d_track.txt' % k, 'w')

        obs_n = env.reset()
        start = time.time()

        for step in range(arglist.display_step_max):

            # 选择动作
            action_n, worldtarget = action(arglist, worldtarget, obs_n, step, NewController)

            new_obs_n, rew_n, done_n, info_n = env.step(action_n)

            obs_n = new_obs_n

            # 保存每个小瓜子每个step的状态信息
            for k in range(arglist.agent_num):
                with open(os.path.dirname(__file__) + '/track/agent_%d_track.txt' % k, 'a') as f:
                    f.write(str(obs_n[k][0])+' '+str(obs_n[k][1])+' '+str(obs_n[k][2])+' '+str(obs_n[k][3])+'\n')

            # 画图展示
            augment_view(arglist, world, NewController)
            env.render()
            print('>>> Num', num, '>>>> step', step)
            time.sleep(0.01)

        time.sleep(1)
        OCR.calculate_coverage(arglist.cover_edge, arglist.agent_num, arglist.display_step_max, num)
        end = time.time()
        interval = round((end - start), 2)
        print('Time Interval ', interval)
