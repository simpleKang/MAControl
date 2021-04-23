# coding=utf-8

#         .--,       .--,
#        ( (  \.---./  ) )
#         '.__/o   o\__.'
#            {=  ^  =}
#             >  -  <
#            /       \
#           //       \\
#          //|   .   |\\
#          "'\       /'"_.-~^`'-.
#             \  _  /--'         `
#           ___)( )(___
#          (((__) (__)))    长烟一空，皓月千里，浮光跃金，_ _ _ _ ，渔歌互答，_ _ _ _ ！

import argparse
import time
import os
import numpy as np
import GeneticAlgorithm.genetic_algorithm_duo as ga_duo
# from GeneticAlgorithm.BehaviorArchetypes import behavior
from GeneticAlgorithm.BehaviorArchetypes_vision import behavior_v

import MAEnv.scenarios.TargetProfile as T
import MAControl.Util.get_random_state as rs

import MAControl.Default.InnerController_PID as IC_P
import MAControl.Default.MotionController_L1_TECS as MC_L
import MAControl.Default.PathPlanner_EdgeWaypoint as PP_G
# import MAControl.Default.PolicyMaker_SOcomm as PM_C
import MAControl.Default.PolicyMaker_SOvisionOP as PM_V

path = '/track/' if os.name == 'posix' else '\\track\\'


def parse_args():

    parser = argparse.ArgumentParser("Control Experiments for Multi-Agent Environments")

    # Environment
    parser.add_argument("--scenario", type=str, default="scenario5_visionOP", help="name of the scenario script")
    parser.add_argument("--uav-num", type=int, default=7, help="number of uav")
    parser.add_argument("--step-max", type=int, default=100, help="number of maximum steps")

    # GA
    parser.add_argument("--pop-size", type=int, default=5, help="size of population")
    parser.add_argument("--preserved-population", type=float, default=0.5, help="percentage of population selected")
    parser.add_argument("--generation-num", type=int, default=4, help="number of generation")
    parser.add_argument("--max-behavior-archetypes", type=int, default=1, help="number of behavior archetypes")
    parser.add_argument("--collect-num", type=int, default=3, help="number of fitness score collection")  # per genratn?

    # Core parameters
    parser.add_argument("--crossover-rate", type=float, default=0.1, help="crossover rate")
    parser.add_argument("--mutation-rate", type=float, default=0.9, help="mutation rate")
    parser.add_argument("--mutation-neighborhood", type=float, default=0.05, help="mutation neighborhood")  # ?

    # Evolve or Test
    parser.add_argument("--evolve", action="store_false", default=True)
    parser.add_argument("--test", action="store_true", default=False)
    parser.add_argument("--restore", action="store_true", default=False)
    parser.add_argument("--repeat-num", type=int, default=1, help="number of repeat runs")  # ?

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

    with open(os.path.dirname(__file__) + path + 'para.txt', 'w') as f:
        f.write(str(arglist.uav_num) + ' ' + str(arglist.step_max))

    return env_, world_, obstacle_info_


def get_controller(env, world, arglist):

    uavController = list()
    targetController = list()

    # 初始化小瓜子
    for i in range(arglist.uav_num):
        control = list()

        control.append(PM_V.PolicyMaker_SO("uav_%d" % i, env, world, i, arglist))
        control.append(PP_G.PathPlanner_EdgeWaypoint("uav_%d" % i, env, world, i, arglist))
        control.append(MC_L.MotionController_L1_TECS("uav_%d" % i, env, world, i, arglist))
        control.append(IC_P.InnerController_PID("uav_%d" % i, env, world, i, arglist))
        control.append(False)  # Arrive-flag

        uavController.append(control)

    # 初始化小花生
    for i in range(T.num_targets):
        control = list()

        # i 是作为target的编号 # i+arglist.uav_num 是作为agent的编号
        control.append(PM_V.PolicyMaker_SO("target_%d" % i, env, world, i+arglist.uav_num, arglist))
        control.append(PP_G.PathPlanner_EdgeWaypoint("target_%d" % i, env, world, i+arglist.uav_num, arglist))
        control.append(MC_L.MotionController_L1_TECS("target_%d" % i, env, world, i+arglist.uav_num, arglist))
        control.append(IC_P.InnerController_PID("target_%d" % i, env, world, i+arglist.uav_num, arglist))
        control.append(False)  # Arrive-flag

        targetController.append(control)

    return uavController, targetController


def action(world, obs_n, step, ControllerSet, obstacles, behavior_archetypes):

    # get action
    action_n = list()

    # 小瓜子或小花生的运动
    for i in range(ControllerSet.__len__()):  # 提取ControllerSet的长度

        list_i = ControllerSet[i][0].\
            make_policy(world, obstacles, obs_n, behavior_archetypes, step)

        pointAi, pointBi, finishedi = ControllerSet[i][1].\
            planpath(list_i, obs_n[i], ControllerSet[i][4], step, obstacles)

        acctEi, acclEi, ControllerSet[i][4] = ControllerSet[i][2]. \
            get_expected_action(obs_n[i], pointAi, pointBi, step, finishedi)

        actioni = ControllerSet[i][3]. \
            get_action(obs_n[i], acctEi, acclEi, step, finishedi)

        action_n.append(actioni)

    return action_n


def augment_view(arglist, world, Controller, obs, step):
    for i in range(arglist.uav_num):
        pass


def get_score(arglist, gen, ind, num):

    # 不使用特定评分，仅给出随机分数
    _score = np.random.random()

    # KSB 像素计算覆盖率
    # _score = calculate.coverrate_k(gen, ind, num)

    # KSB 像素计算覆盖率 - 视觉
    # _score = cv.calculate_coverage(arglist.uav_num, arglist.step_max, num)

    # WZQ 完整计算覆盖率方式
    # _score = OCR.calculate_coverage(arglist.uav_num, arglist.step_max, num)

    # WZQ 简易计算覆盖率方式
    # _score = cc.calculate_coverage(arglist.uav_num, arglist.step_max, num)

    # WZQ 最小化攻击时间间隔作为评分
    # target = np.loadtxt(os.path.dirname(__file__) + path + 'target_attacking.txt')
    # if target.size > 3:
    #     x, y = target.shape
    #     if target[x-1][y-1] == 0:
    #         _score = target[0][0] - target[x-1][0]
    #     else:
    #         _score = -arglist.step_max
    # else:
    #     _score = -arglist.step_max

    # WZQ 计算目标处于UAV感知范围内的时间
    # _score = round(sum(PM_S.PolicyMaker_SO.target_in_sight) / arglist.uav_num, 2)
    # PM_S.PolicyMaker_SO.target_in_sight.clear()

    # WZQ 有限数量目标吸引，多于排斥
    # target_seen_step = np.array(PM_S.PolicyMaker_SO.target_in_sight[:arglist.uav_num])
    # step_sum = np.sum(target_seen_step, axis=0)
    # reward = 0
    # punish = 0
    # for i in range(step_sum.size):
    #     if step_sum[i] > 0:
    #         reward += 1
    #         if step_sum[i] > T.target_H[0]:
    #             punish += 1
    # _score = reward - punish
    # PM_S.PolicyMaker_SO.target_in_sight.clear()

    # WZQ 目标吸引加权得分，排斥扣分
    # target_seen_step = np.array(PM_V.PolicyMaker_SO.target_in_sight[:arglist.uav_num])
    # step_sum = np.sum(target_seen_step, axis=0)
    # _score = 0
    # for i in range(step_sum.size):
    #     if 0 < step_sum[i] <= T.target_H[0]:
    #         _score += 1 - 0.3*(T.target_H[0] - step_sum[i])
    #     elif step_sum[i] > T.target_H[0]:
    #         _score -= 1
    # PM_V.PolicyMaker_SO.target_in_sight.clear()

    # WZQ 视觉列队飞行
    # uav_seen_step = np.array(PM_V.PolicyMaker_SO.uav_in_sight[:arglist.uav_num])
    # step_sum = np.sum(uav_seen_step, axis=0)
    # _score = sum(step_sum)
    # PM_V.PolicyMaker_SO.uav_in_sight.clear()

    return _score


def run_simulation(arglist, behavior_archetypes, gen, ind, num):

    with open(os.path.dirname(__file__) + path + 'para.txt', 'w') as f:
        f.write(str(arglist.uav_num) + ' ' + str(arglist.step_max) + ' ' + str(num))

    # Create environment
    env, world, obstacle_info = make_env(arglist)

    # Create Controller
    Controllers = get_controller(env, world, arglist)

    open(os.path.dirname(__file__) + path + 'target_attacking.txt', 'w')

    # 为每个小瓜子创建状态文件
    for k in range(arglist.uav_num):
        open(os.path.dirname(__file__) + path + 'uav_%d_track.txt' % k, 'w')

    obs_n = env.reset()

    for step in range(arglist.step_max):

        # 选择动作
        action_Un = action(world, obs_n, step, Controllers[0], obstacle_info, behavior_archetypes)
        action_Tn = action(world, obs_n, step, Controllers[1], obstacle_info, behavior_archetypes)
        action_n = action_Un + action_Tn

        new_obs_n, rew_n, done_n, info_n = env.step(action_n)

        obs_n = new_obs_n

        # 保存每个小瓜子每个step的状态信息
        for k in range(arglist.uav_num):
            with open(os.path.dirname(__file__) + path + 'uav_%d_track.txt' % k, 'a') as f:
                f.write(str(obs_n[k][0]) + ' ' + str(obs_n[k][1]) + ' ' +
                        str(obs_n[k][2]) + ' ' + str(obs_n[k][3]) + '\n')

        # print('>>> Step ', step)

        # 画图展示
        augment_view(arglist, world, Controllers[0], obs_n, step)
        # env.render()
        time.sleep(0.001)

    time.sleep(0.1)
    _score = get_score(arglist, gen, ind, num)

    return _score


if __name__ == '__main__':

    arglist = parse_args()
    r_state = rs.RandomState(arglist.collect_num, arglist.uav_num)
    T.init_state = r_state

    if arglist.evolve:
        ga = ga_duo.GA(arglist)

        for gen in range(arglist.generation_num + 1):
            for ind, individual in enumerate(ga.population):
                for num in range(arglist.collect_num):

                    start = time.time()
                    score = run_simulation(arglist, individual, gen, ind, num)
                    end = time.time()
                    print('>>> Generation', gen, '>>> Individual', ind, '>>> Collect', num,
                          'time-consuming: ', round((end - start), 2), 'score: ', score)
                    ga.score[ind][num] = score

            ga.save_pop(gen)

            if gen < arglist.generation_num:
                ga.evolve(gen)

        print('All finished!')

    if arglist.test:

        print('Test with specific behavior archetypes.')
        for repeat in range(arglist.repeat_num):
            score = run_simulation(arglist, behavior_v, 0, 0, repeat)
            print('score ', score)

