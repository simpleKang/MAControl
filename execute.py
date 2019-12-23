# coding=utf-8

import argparse
import numpy as np
import time
import os
import Trainer.DQN_trainer as T
import Trainer.update_train_info as ut
import MAControl.Util.OfflineCoverRate as OCR

import MAControl.Test_Auction.InnerController_PID as IC_P
import MAControl.Test_Auction.MotionController_L1_TECS as MC_L
import MAControl.Test_Auction.PathPlanner_EgdeWaypoint as PP_G
import MAControl.Test_Auction.PolicyMaker_LocalDecision as PM_L


def parse_args():

    parser = argparse.ArgumentParser("Control Experiments for Multi-Agent Environments")

    # Environment
    parser.add_argument("--scenario", type=str, default="scenario5_dqn", help="name of the scenario script")
    parser.add_argument("--agent-num", type=int, default=20, help="number of agent")
    parser.add_argument("--train-step-max", type=int, default=10000, help="number of episodes")
    parser.add_argument("--episode-step-max", type=int, default=4000, help="maximum episode length")
    parser.add_argument("--display-step-max", type=int, default=4000, help="number of episodes for displaying")
    parser.add_argument("--learn-num", type=int, default=50, help="number of learning rounds after collecting data")
    parser.add_argument("--data-collect-num", type=int, default=1, help="number of data collector")
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

        control.append(PM_L.PolicyMaker_LocalDecision("agent_%d" % i, env, world, i, arglist))
        control.append(PP_G.PathPlanner_EgdeWaypoint("agent_%d" % i, env, world, i, arglist))
        control.append(MC_L.MotionController_L1_TECS("agent_%d" % i, env, world, i, arglist))
        control.append(IC_P.InnerController_PID("agent_%d" % i, env, world, i, arglist))
        control.append(False)  # Arriveflag
        control.append(False)  # Isattacking

        ControllerSet.append(control)

    return ControllerSet


def action(arglist, WorldTarget, obs_n, step, NewController, trainer_):

    # get action
    action_n = list()

    # 小瓜子运动
    for i in range(arglist.agent_num):

        list_i, sight_friends = NewController[i][0]. \
            make_policy(WorldTarget, obs_n, step, trainer_)

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
    trainer = T.DQN_trainer(env, world, arglist, n_actions=len(ut.action_dict), n_features=40)

    # Create Controller
    NewController = get_controller(env, world, arglist)

    if arglist.train:

        open(os.path.dirname(__file__) + '/save_model/cost.txt', 'w')

        for episode in range(arglist.train_step_max):

            obs_n = env.reset()
            start = time.time()

            for episode_step in range(arglist.episode_step_max):

                # env.render()

                # 根据网络选择动作
                action_n, worldtarget, store_state = ut.update_next_state(arglist, worldtarget, obs_n,
                                                                          episode_step, NewController, trainer)
                new_obs_n, rew_n, done_n, info_n = env.step(action_n)

                if store_state:

                    reward = ut.update_reward_6(store_state)

                    is_store = ut.update_storage(arglist, store_state, new_obs_n, reward, trainer)

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

                # 选择动作
                action_n, worldtarget = action(arglist, worldtarget, obs_n, step, NewController, trainer)

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
                time.sleep(0.01)

            time.sleep(1)
            OCR.calculate_coverage(arglist.cover_edge, arglist.agent_num, arglist.display_step_max, num)
            end = time.time()
            interval = round((end - start), 2)
            print('Time Interval ', interval)
