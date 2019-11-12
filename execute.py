# coding=utf-8

import argparse
import numpy as np
import time
import os
import Trainer.DQN_trainer as T
import MAControl.Util.OnlineCoverRate as CR

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


# [w1, w2, w3, w4]
action_dict = {"0": [1., 0., 0., 0.],
               "1": [0., 1., 0., 0.],
               "2": [0., 0., 1., 0.],
               "3": [0., 0., 0., 1.]}


def parse_args():
    parser = argparse.ArgumentParser("Control Experiments for Multi-Agent Environments")

    # Environment
    parser.add_argument("--scenario", type=str, default="scenario5_dqn", help="name of the scenario script")
    parser.add_argument("--episode-step-max", type=int, default=100, help="maximum episode length")
    parser.add_argument("--train-step-max", type=int, default=2000, help="number of episodes")
    parser.add_argument("--display-step-max", type=int, default=10000, help="number of episodes for displaying")
    parser.add_argument("--cover-edge", type=int, default=200, help="number of cells of one edge")
    parser.add_argument("--agent-num", type=int, default=10, help="number of agent")
    parser.add_argument("--OnlineCoverRate", type=int, default=0, help="Online or offline to calculate cover rate")

    # Core training parameters
    parser.add_argument("--lr", type=float, default=1e-2, help="learning rate for Adam optimizer")
    parser.add_argument("--gamma", type=float, default=0.9, help="discount factor")
    parser.add_argument("--batch-size", type=int, default=50, help="sample batch memory from all memory for training")
    parser.add_argument("--replace-target-iter", type=int, default=300, help="copy eval net params into target net")

    # Checkpointing
    parser.add_argument("--save-dir", type=str, default="./save_model/model",
                        help="directory in which training state and model should be saved")
    parser.add_argument("--save-rate", type=int, default=100,
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
    for i in range(env.n - len(world.movable_targets)):
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


def update_action(env, world, WorldTarget, obs_n, step, NewController, w):

    # get action
    action_n = list()

    # 小瓜子运动
    for i in range(env.n - len(world.movable_targets)):

        list_i = NewController[i][0]. \
            make_policy(WorldTarget, obs_n, step)

        if list_i[0] != 0:
            v = np.array([0., 0.])
            for x in range(4):
                list_i[1][x] *= w[i][x]
                v += list_i[1][x]
            list_i[1] = v

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
    trainer = T.DQN_trainer(env, world, arglist, n_actions=len(action_dict))

    # Create Controller
    NewController = get_controller(env, world, arglist)

    if arglist.train:

        for episode in range(arglist.train_step_max):

            obs_n = env.reset()
            start = time.time()
            agent_index = np.random.randint(0, arglist.agent_num)

            for episode_step in range(arglist.episode_step_max):

                # env.render()
                w_env = list()
                # TODO 具体状态值待重设
                w = trainer.choose_action(obs_n)
                for act in w:
                    w_env.append(action_dict[str(int(act))])
                action_n, worldtarget = update_action(env, world, worldtarget, obs_n, episode_step, NewController, w_env)
                new_obs_n, rew_n, done_n, info_n = env.step(action_n)
                w = np.reshape(w, [arglist.agent_num, 1])
                rew_n = np.reshape(rew_n, [arglist.agent_num, 1])
                trainer.store_transition(obs_n[agent_index],
                                         w[agent_index],
                                         rew_n[agent_index],
                                         new_obs_n[agent_index])
                obs_n = new_obs_n

            trainer.learn()
            end = time.time()
            if episode % arglist.save_rate == 0:
                trainer.save_model(arglist.save_dir, episode)
            print("Train_Step:", episode)
            print("cost:", trainer.cost_his[-1])

    if arglist.display:

        with open(os.path.dirname(__file__) + '/track/para.txt', 'w') as f:
            f.write(str(arglist.cover_edge) + '\n')
            f.write(str(arglist.agent_num) + '\n')

        # 为每个小瓜子创建路径文件
        for k in range(env.n - len(world.movable_targets)):
            open(os.path.dirname(__file__) + '/track/agent_%d_track.txt' % k, 'w')

        step = 0

        obs_n = env.reset()

        start = time.time()

        # if arglist.OnlineCoverRate:
        #     last_cover = list()
        #     area = np.zeros((arglist.cover_edge, arglist.cover_edge))
        #     open('cover_rate.txt', 'w')
        #     for k in range(env.n - len(world.movable_targets)):
        #         last_cover.append([])

        while True:

            w_env = list()
            w = trainer.choose_action(obs_n)
            for act in w:
                w_env.append(action_dict[str(int(act))])

            # get action
            action_n, worldtarget = update_action(env, world, worldtarget, obs_n, step, NewController, w_env)

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
            # env.render()
            if step > arglist.display_step_max:
                end = time.time()
                interval = round((end - start), 2)
                print('Time Interval ', interval)
                break
            print('>>>> step', step)
