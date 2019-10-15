# coding=utf-8

import argparse
import numpy as np
import time
import MAControl.Util.CoverRate as CR
import MAControl.Test_Auction.InnerController_PID as IC_P
import MAControl.Test_Auction.MotionController_L1_TECS as MC_L
import MAControl.Test_Auction.PathPlanner_Simple as PP_S
import MAControl.Test_Auction.PathPlanner_generate_at_present as PP_G
import MAControl.Test_Auction.PolicyMaker_Auction as PM_A
import MAControl.Test_Movable_Target_Policy.InnerController_PID as T_IC_P
import MAControl.Test_Movable_Target_Policy.MotionController_L1_TECS as T_MC_L
import MAControl.Test_Movable_Target_Policy.PathPlanner_Simple as T_PP_S
import MAControl.Test_Movable_Target_Policy.PolicyMaker_Auction as T_PM_A


def parse_args():
    parser = argparse.ArgumentParser("Control Experiments for Multi-Agent Environments")
    parser.add_argument("--scenario", type=str, default="scenario4_Xuxiao", help="name of the scenario script")
    parser.add_argument("--step-max", type=int, default=4000, help="maximum steps")
    parser.add_argument("--track_file", type=str, default="/home/wzq/pycode/MAC/MAControl/track", help="file path of agent's track")
    parser.add_argument("--cover_edge", type=int, default=200, help="number of cells of one edge")
    return parser.parse_args()


def make_env(arglist):
    from MAEnv.environment import MultiAgentEnv
    import MAEnv.scenarios as scenarios

    # load scenario from script
    scenario = scenarios.load(arglist.scenario + ".py").Scenario()

    # create world and env
    world = scenario.make_world()
    env = MultiAgentEnv(world, scenario.reset_world, scenario.reward, scenario.observation)
    return env, world


def get_controller(env, world, arglist):
    ControllerSet = []

    # 初始化小瓜子
    for i in range(env.n - len(world.movable_targets)):
        control = []
        control.append(PM_A.PolicyMaker_Auction("agent_%d" % i, env, world, i, arglist))
        control.append(PP_S.PathPlanner_Simple("agent_%d" % i, env, world, i, arglist))
        # control.append(PP_G.PathPLanner_generate_at_present("agent_%d" % i, env, world, i, arglist))
        control.append(MC_L.MotionController_L1_TECS("agent_%d" % i, env, world, i, arglist))
        control.append(IC_P.InnerController_PID("agent_%d" % i, env, world, i, arglist))
        control.append(False)  # Arriveflag
        control.append(False)  # Isattacking
        ControllerSet.append(control)

    # 初始化动目标
    for i in range(len(world.movable_targets)):
        control = []
        control.append(T_PM_A.PolicyMaker_Target("movable_target_%d" % i, env, world, i, arglist))
        control.append(T_PP_S.PathPlanner_Simple("movable_target_%d" % i, env, world, i, arglist))
        control.append(T_MC_L.MotionController_L1_TECS("movable_target_%d" % i, env, world, i, arglist))
        control.append(T_IC_P.InnerController_PID("movable_target_%d" % i, env, world, i, arglist))
        control.append(False)  # Arriveflag
        control.append(False)  # Isattacking
        ControllerSet.append(control)

    return ControllerSet


def update_action(env, world, obs_n, step, NewController):

    # WorldTarget
    WorldTarget = []
    for i, landmark in enumerate(world.targets):
        WorldTarget.append([landmark.state.p_pos[0], landmark.state.p_pos[1], landmark.state.p_vel[0],
                            landmark.state.p_vel[1], landmark.value, landmark.defence])

    # get action
    action_n = []

    # 小瓜子运动
    for i in range(env.n - len(world.movable_targets)):

        list_i = NewController[i][0]. \
            make_policy(WorldTarget, obs_n, step)

        pointAi, pointBi, finishedi, NewController[i][5] = NewController[i][1].\
            planpath(list_i, obs_n[i], NewController[i][4], step)

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

    return action_n


def augment_view(env, world, NewController):
    for i in range(env.n):
        if NewController[i][5]:
            world.agents[i].attacking = True


if __name__ == '__main__':
    arglist = parse_args()

    # Create environment
    env, world = make_env(arglist)

    # Create Controller
    NewController = get_controller(env, world, arglist)

    obs_n = env.reset()
    step = 0
    start = time.time()

    # # 为每个小瓜子创建路径文件
    # for k in range(env.n - len(world.movable_targets)):
    #     open(arglist.track_file+'/agent_%d_track.txt' % k, 'w')
    open('cover_rate.txt', 'w')

    # 区域离散化
    area = np.zeros((arglist.cover_edge, arglist.cover_edge))

    while True:

        # get action
        print('>>>> step', step)
        action_n = update_action(env, world, obs_n, step, NewController)

        # environment step
        new_obs_n, rew_n, done_n, info_n = env.step(action_n)
        step += 1
        obs_n = new_obs_n

        if step % 5 == 0:
            area = CR.update_area_cover(arglist.cover_edge, area, obs_n, env.n - len(world.movable_targets))
            cover_rate = CR.cal_cover_rate(area)
            with open('cover_rate.txt', 'a') as c:
                c.write(str(step)+' '+str(cover_rate)+'\n')

        # # 记录每个小瓜子每个step的位置
        # for k in range(env.n - len(world.movable_targets)):
        #     with open(arglist.track_file+'/agent_%d_track.txt' % k, 'a') as f:
        #         f.write(str(obs_n[k][2])+' '+str(obs_n[k][3])+'\n')

        # for displaying
        # time.sleep(0.01)
        augment_view(env, world, NewController)
        env.render()
        print('>>>> step', step)
