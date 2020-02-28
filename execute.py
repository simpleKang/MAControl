# coding=utf-8

import argparse
import time
import os
import MAEnv.scenarios.TargetProfile as T

import MAControl.Default.InnerController_PID as IC_P
import MAControl.Default.MotionController_L1_TECS as MC_L
import MAControl.Default.PathPlanner_EdgeWaypoint as PP_G
import MAControl.Default.PolicyMaker_SelfOrganization as PM_S


def parse_args():

    parser = argparse.ArgumentParser("Control Experiments for Multi-Agent Environments")

    # Environment
    parser.add_argument("--scenario", type=str, default="scenario6_AFIT", help="name of the scenario script")
    parser.add_argument("--uav-num", type=int, default=1, help="number of uav")
    parser.add_argument("--step-max", type=int, default=4000, help="number of maximum steps")
    parser.add_argument("--repeat-num", type=int, default=1, help="number of repeat runs")

    return parser.parse_args()


def make_env(arglist):
    from MAEnv.environment import MultiAgentEnv
    import MAEnv.scenarios as scenarios

    # load scenario from script
    scenario = scenarios.load(arglist.scenario + ".py").Scenario()

    # create world and env
    world_ = scenario.make_World(arglist.uav_num)
    env_ = MultiAgentEnv(world_, scenario.reset_world, scenario.reward, scenario.observation)

    # creat obstacle_info_
    obstacle_info_ = list()
    for k, landmark in enumerate(world_.landmarks):
        if landmark.obstacle:
            obstacle_info_.append([landmark.state.p_pos[0], landmark.state.p_pos[1], landmark.size, k])

    return env_, world_, obstacle_info_


def get_controller(env, world, arglist):

    uavController = list()
    targetController = list()

    # 初始化小瓜子
    for i in range(arglist.uav_num):
        control = list()

        control.append(PM_S.PolicyMaker_SelfOrganization("uav_%d" % i, env, world, i, arglist))
        control.append(PP_G.PathPlanner_EdgeWaypoint("uav_%d" % i, env, world, i, arglist))
        control.append(MC_L.MotionController_L1_TECS("uav_%d" % i, env, world, i, arglist))
        control.append(IC_P.InnerController_PID("uav_%d" % i, env, world, i, arglist))
        control.append(False)  # Arriveflag

        uavController.append(control)

    # 初始化小花生
    for i in range(T.num_targets):
        control = list()

        # i 是作为target的编号 # i+arglist.uav_num 是作为agent的编号
        control.append(PM_S.PolicyMaker_SelfOrganization("target_%d" % i, env, world, i+arglist.uav_num, arglist))
        control.append(PP_G.PathPlanner_EdgeWaypoint("target_%d" % i, env, world, i+arglist.uav_num, arglist))
        control.append(MC_L.MotionController_L1_TECS("target_%d" % i, env, world, i+arglist.uav_num, arglist))
        control.append(IC_P.InnerController_PID("target_%d" % i, env, world, i+arglist.uav_num, arglist))
        control.append(False)  # Arriveflag

        targetController.append(control)

    return uavController, targetController


def action(obs_n, step, ControllerSet, obstacles):

    # get action
    action_n = list()

    # 小瓜子或小花生的运动
    for i in range(ControllerSet.__len__()):  # 提取ControllerSet的长度

        list_i = ControllerSet[i][0].\
            make_policy(obstacles, obs_n, step)

        pointAi, pointBi, finishedi = ControllerSet[i][1].\
            planpath(list_i, obs_n[i], ControllerSet[i][4], step, obstacles)

        acctEi, acclEi, ControllerSet[i][4] = ControllerSet[i][2]. \
            get_expected_action(obs_n[i], pointAi, pointBi, step, finishedi)

        actioni = ControllerSet[i][3]. \
            get_action(obs_n[i], acctEi, acclEi, step, finishedi)

        action_n.append(actioni)

    return action_n


if __name__ == '__main__':

    arglist = parse_args()

    # Create environment
    env, world, obstacle_info = make_env(arglist)

    # Create Controller
    Controllers = get_controller(env, world, arglist)

    for num in range(arglist.repeat_num):

        with open(os.path.dirname(__file__) + '/track/para.txt', 'w') as f:
            f.write(str(T.edge) + ' ' + str(arglist.uav_num) + ' ' + str(arglist.step_max))
        # 为每个小瓜子创建状态文件
        for k in range(arglist.uav_num):
            open(os.path.dirname(__file__) + '/track/uav_%d_track.txt' % k, 'w')

        obs_n = env.reset()
        start = time.time()

        for step in range(arglist.step_max):

            # 选择动作
            # action_Un = action(obs_n[0:arglist.uav_num], step, Controllers[0], obstacle_info)
            action_Un = action(obs_n, step, Controllers[0], obstacle_info)
            action_Tn = action(obs_n[arglist.uav_num:], step, Controllers[1], obstacle_info)
            action_n = action_Un + action_Tn

            new_obs_n, rew_n, done_n, info_n = env.step(action_n)

            obs_n = new_obs_n

            # 保存每个小瓜子每个step的状态信息
            for k in range(arglist.uav_num):
                with open(os.path.dirname(__file__) + '/track/uav_%d_track.txt' % k, 'a') as f:
                    f.write(str(obs_n[k][0]) + ' ' + str(obs_n[k][1]) + ' ' + str(obs_n[k][2]) + ' ' + str(
                        obs_n[k][3]) + '\n')

            # 画图展示
            env.render()
            print('>>> Num', num, '>>>> step', step)
            time.sleep(0.01)

        time.sleep(1)
        end = time.time()
        interval = round((end - start), 2)
        print('Time Interval ', interval)
