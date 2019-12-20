# coding=utf-8

import argparse
import numpy as np
import time
import os
import Trainer.DQN_trainer as T
import Trainer.update_reward as ur
import Trainer.update_action as ua
import MAControl.Util.OfflineCoverRate as OCR

import MAControl.Test_Auction.InnerController_PID as IC_P
import MAControl.Test_Auction.MotionController_L1_TECS as MC_L

import MAControl.Test_Auction.PathPlanner_Simple as PP_S
import MAControl.Test_Auction.PathPlanner_generate_at_present as PP_G

import MAControl.Test_Auction.PolicyMaker_Auction as PM_A
import MAControl.Test_Auction.PolicyMaker_Weight as PM_W
import MAControl.Test_Auction.PolicyMaker_Weight_T as PM_T
import MAControl.Test_Auction.PolicyMaker_Weight_V as PM_V

import MAControl.Test_Movable_Target_Policy.InnerController_PID as T_IC_P
import MAControl.Test_Movable_Target_Policy.MotionController_L1_TECS as T_MC_L
import MAControl.Test_Movable_Target_Policy.PathPlanner_Simple as T_PP_S
import MAControl.Test_Movable_Target_Policy.PolicyMaker_Auction as T_PM_A


def parse_args():

    parser = argparse.ArgumentParser("Control Experiments for Multi-Agent Environments")

    # Environment
    parser.add_argument("--scenario", type=str, default="scenario5_dqn", help="name of the scenario script")
    parser.add_argument("--agent-num", type=int, default=20, help="number of agent")
    parser.add_argument("--train-step-max", type=int, default=20, help="number of episodes")
    parser.add_argument("--episode-step-max", type=int, default=4000, help="maximum episode length")
    parser.add_argument("--display-step-max", type=int, default=4000, help="number of episodes for displaying")
    parser.add_argument("--learn-num", type=int, default=50, help="number of learning rounds after collecting data")
    parser.add_argument("--data-collect-num", type=int, default=30, help="number of data collector")
    parser.add_argument("--cover-edge", type=int, default=200, help="number of cells of one edge")

    # Core training parameters
    parser.add_argument("--lr", type=float, default=1e-3, help="learning rate for Adam optimizer")
    parser.add_argument("--gamma", type=float, default=0.9, help="discount factor")
    parser.add_argument("--e-greedy", type=float, default=0.9, help="epsilon greedy")
    parser.add_argument("--e-greedy-max", type=float, default=0.99, help="max of epsilon greedy")
    parser.add_argument("--e-greedy-increment", type=float, default=1e-3, help="increment of epsilon greedy")
    parser.add_argument("--batch-size", type=int, default=50, help="sample batch memory from all memory for training")
    parser.add_argument("--memory-size", type=int, default=1000000, help="memory for training")
    parser.add_argument("--replace-target-iter", type=int, default=300, help="copy eval net params into target net")
    parser.add_argument("--use-doubleQ", action="store_false", default=True, help="whether to use double Q net")

    # Checkpointing
    parser.add_argument("--save-dir", type=str, default="./save_model/model",
                        help="directory in which training state and model should be saved")
    parser.add_argument("--save-rate", type=int, default=5,
                        help="save model once every time this many episodes are completed")
    parser.add_argument("--load-dir", type=str, default="./save_model/",
                        help="directory in which training state and model are loaded")

    # Evaluation
    parser.add_argument("--restore", action="store_true", default=False)
    parser.add_argument("--display", action="store_true", default=False)
    parser.add_argument("--train", action="store_false", default=True)

    return parser.parse_args()


def make_env(arglist):
    from MAEnv.environment import MultiAgentEnv
    import MAEnv.scenarios as scenarios

    # load scenario from script
    scenario = scenarios.load(arglist.scenario + ".py").Scenario()

    # create world and env
    world_ = scenario.make_world(arglist.agent_num)
    env_ = MultiAgentEnv(world_, scenario.reset_world, scenario.reward, scenario.observation)

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
        # control.append(PM_A.PolicyMaker_Auction("agent_%d" % i, env, world, i, arglist))
        # control.append(PM_W.PolicyMaker_Weight("agent_%d" % i, env, world, i, arglist))
        # control.append(PM_T.PolicyMaker_Weight_T("agent_%d" % i, env, world, i, arglist))
        control.append(PM_V.PolicyMaker_Weight_V("agent_%d" % i, env, world, i, arglist))

        # control.append(PP_S.PathPlanner_Simple("agent_%d" % i, env, world, i, arglist))
        control.append(PP_G.PathPlanner_generate_at_present("agent_%d" % i, env, world, i, arglist))

        control.append(MC_L.MotionController_L1_TECS("agent_%d" % i, env, world, i, arglist))
        control.append(IC_P.InnerController_PID("agent_%d" % i, env, world, i, arglist))
        control.append(False)  # Arriveflag
        control.append(False)  # Isattacking
        ControllerSet.append(control)

    # 初始化动目标
    # for i in range(len(world.movable_targets)):
    #     control = list()
    #     control.append(T_PM_A.PolicyMaker_Target("movable_target_%d" % i, env, world, i, arglist))
    #     control.append(T_PP_S.PathPlanner_Simple("movable_target_%d" % i, env, world, i, arglist))
    #     control.append(T_MC_L.MotionController_L1_TECS("movable_target_%d" % i, env, world, i, arglist))
    #     control.append(T_IC_P.InnerController_PID("movable_target_%d" % i, env, world, i, arglist))
    #     control.append(False)  # Arriveflag
    #     control.append(False)  # Isattacking
    #     ControllerSet.append(control)

    return ControllerSet


def specific_w_action(env, world, WorldTarget, obs_n, step, NewController, w):

    def update_w(list_i, w):

        if len(list_i[1]) == len(w):
            v = 0
            for i in range(len(w)):
                v += list_i[1][i] * w[i]
            list_i[1] = v

        return list_i

    # get action
    action_n = list()

    # 小瓜子运动
    for i in range(env.n - len(world.movable_targets)):

        list_i = NewController[i][0]. \
            make_policy(WorldTarget, obs_n, step)

        list_i = update_w(list_i, w)

        pointAi, pointBi, finishedi, NewController[i][5], WorldTarget = NewController[i][1].\
            planpath(list_i, obs_n[i], NewController[i][4], step, WorldTarget)

        acctEi, acclEi, NewController[i][4] = NewController[i][2]. \
            get_expected_action(obs_n[i], pointAi, pointBi, step, finishedi)

        actioni = NewController[i][3]. \
            get_action(obs_n[i], acctEi, acclEi, step, finishedi)

        action_n.append(actioni)

    # 动目标运动
    # for i in range(env.n - len(world.movable_targets), env.n):
    #
    #     list_i = NewController[i][0]. \
    #         make_policy(WorldTarget, obs_n, step)
    #
    #     pointAi, pointBi, finishedi, NewController[i][5] = NewController[i][1]. \
    #         planpath(list_i, obs_n[i], NewController[i][4], step)
    #
    #     acctEi, acclEi, NewController[i][4] = NewController[i][2]. \
    #         get_expected_action(obs_n[i], pointAi, pointBi, step, finishedi)
    #
    #     actioni = NewController[i][3]. \
    #         get_action(obs_n[i], acctEi, acclEi, step, finishedi)
    #
    #     action_n.append(actioni)

    return action_n, WorldTarget


def store_valuable_state(arglist, episode_step_, new_obs_n_, cache_obs_, cache_w_, cache_rew_):

    # 更新每个个体的 new_bos
    new_obs_cache = list()
    for agent in range(arglist.agent_num):
        new_obs_temp = new_obs_n_[agent].copy()
        for v1_4 in v_set[agent]:
            new_obs_temp = np.concatenate([new_obs_temp] + [v1_4])

        # 删除多余状态信息 速度/加速度
        new_obs_temp = np.delete(new_obs_temp, [0, 1, 4, 5, 6, 7], axis=0)

        new_obs_cache.append(new_obs_temp)

    # 存储有价值的个体的 obs / w / reward / new_obs
    if episode_step_ > 0:
        for agent in range(arglist.agent_num):
            v2_not_0 = np.sum(v_set[agent][1])
            v3_not_0 = np.sum(v_set[agent][2])
            v4_not_0 = np.sum(v_set[agent][3])
            if v2_not_0 != 0 or v3_not_0 != 0 or v4_not_0 != 0:
                trainer.store_transition(cache_obs_[agent],
                                         np.array([cache_w_[agent]]),
                                         cache_rew_,
                                         new_obs_cache[agent])

    # 更新缓存区
    cache_obs_ = new_obs_cache.copy()
    cache_w_ = w.copy()
    cache_rew_ = np.array([reward])

    return cache_obs_, cache_w_, cache_rew_


def augment_view(arglist, world, NewController):
    for i in range(arglist.agent_num):
        if NewController[i][5]:
            world.agents[i].attacking = True


if __name__ == '__main__':

    arglist = parse_args()

    # Create environment
    env, world, worldtarget = make_env(arglist)
    # 手动输入状态维度 n_features
    trainer = T.DQN_trainer(env, world, arglist, n_actions=len(ua.action_dict), n_features=10)

    # Create Controller
    NewController = get_controller(env, world, arglist)

    if arglist.train:

        open(os.path.dirname(__file__) + '/save_model/cost.txt', 'w')
        # open(os.path.dirname(__file__) + '/save_model/reward.txt', 'w')
        median = np.loadtxt('reflection_median.txt')

        for episode in range(arglist.train_step_max):

            obs_n = env.reset()
            start = time.time()

            # 初始化全局 reward
            last_cover = list()
            area = np.zeros((arglist.cover_edge, arglist.cover_edge))
            for k in range(arglist.agent_num):
                last_cover.append([])

            # 初始化缓存区
            cache_obs = None
            cache_w = None
            cache_rew = None

            for episode_step in range(arglist.episode_step_max):

                # env.render()

                # 根据网络选择动作
                action_n, worldtarget, v_set, w = ua.net_choose_action_w(arglist, worldtarget, obs_n, episode_step, NewController, trainer)
                new_obs_n, rew_n, done_n, info_n = env.step(action_n)

                # 计算全局 reward
                # reward, area, last_cover = ur.update_reward_1(arglist, area, last_cover, obs_n, episode_step)
                # reward, area, last_cover = ur.update_reward_2(arglist, area, last_cover, obs_n, median[episode_step])
                # reward, area, last_cover = ur.update_reward_3(arglist, area, last_cover, obs_n, median[episode_step])
                # reward, area, last_cover = ur.update_reward_4(arglist, area, last_cover, obs_n)
                reward, area, last_cover = ur.update_reward_5(arglist, area, last_cover, obs_n, episode_step, median)

                # with open(os.path.dirname(__file__) + '/save_model/reward.txt', 'a') as f:
                #     f.write(str(episode_step)+' '+str(reward)+'\n')

                # 按规则存储数据
                cache_obs, cache_w, cache_rew = store_valuable_state(arglist, episode_step, new_obs_n, cache_obs, cache_w, cache_rew)

                obs_n = new_obs_n

                print("Train Step:", episode, " Episode Step:", episode_step)

            for train in range(arglist.learn_num):
                trainer.learn()
                end = time.time()
                print("cost:", trainer.cost_his[-1])
                with open(os.path.dirname(__file__) + '/save_model/cost.txt', 'a') as f:
                    f.write(str(trainer.learn_step_counter)+' '+str(trainer.cost_his[-1])+'\n')

    if arglist.display:

        for num in range(arglist.data_collect_num):

            with open(os.path.dirname(__file__) + '/track/para.txt', 'w') as f:
                f.write(str(arglist.cover_edge)+' '+str(arglist.agent_num)+' '+str(arglist.display_step_max))

            # 为每个小瓜子创建状态文件
            for k in range(arglist.agent_num):
                open(os.path.dirname(__file__) + '/track/agent_%d_track.txt' % k, 'w')

            obs_n = env.reset()
            start = time.time()

            for step in range(arglist.display_step_max):

                # 根据网络选择动作
                action_n, worldtarget, v_set, w = ua.net_choose_action_w(arglist, worldtarget, obs_n, step,
                                                                         NewController, trainer)

                # 自定义动作
                # w = [1, 0, 0, 0]
                # action_n, worldtarget = specific_w_action(env, world, worldtarget, obs_n, step, NewController, w)

                new_obs_n, rew_n, done_n, info_n = env.step(action_n)

                obs_n = new_obs_n

                # 保存每个小瓜子每个step的状态信息
                for k in range(arglist.agent_num):
                    with open(os.path.dirname(__file__) + '/track/agent_%d_track.txt' % k, 'a') as f:
                        f.write(str(obs_n[k][0])+' '+str(obs_n[k][1])+' '+str(obs_n[k][2])+' '+str(obs_n[k][3])+'\n')

                # 画图展示
                augment_view(arglist, world, NewController)
                # env.render()
                print('>>> Num', num, '>>>> step', step)

            OCR.calculate_coverage(arglist.cover_edge, arglist.agent_num, arglist.display_step_max, num)
            end = time.time()
            interval = round((end - start), 2)
            print('Time Interval ', interval)
