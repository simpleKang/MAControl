# coding=utf-8

import argparse
import time
import MAControl.PTMA.InnerController_PID as IC_P
import MAControl.PTMA.MotionController_L1_TECS_6DOF as MC_L
import MAControl.PTMA.PathPlanner_Simple as PP_S
import MAControl.PTMA.PolicyMaker_Probability as PM_P
import logging
import numpy as np

# 运行 execute.py 需要补足参数，如 execute_all.py 中所示
# logging.basicConfig(filename='/home/samantha/gitr/logs/paper/R/result_all.log', level=logging.INFO)
logging.basicConfig(filename='/S-Projects/s-logs/result1.log', level=logging.INFO)
logging.info('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>')
logging.info(time.strftime('%Y-%m-%d, %H:%M:%S'))
logging.info('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>')
# 需要自行指定为本地存在的绝对路径，指定名称的文件如果不存在，会自动创建
# 如果存在，不会覆盖内容，会从结尾处向后添加新内容


def parse_args():
    parser = argparse.ArgumentParser("Control Experiments for Multi-Agent Environments")
    parser.add_argument("--scenario", type=str, default="scenario3d_paper", help="name of the scenario script")
    parser.add_argument("--step-max", type=int, default=8000, help="maximum steps")
    parser.add_argument("--episode-max", type=int, default=10, help="maximum episodes")
    parser.add_argument("--p1", action='append', type=float, dest='p1', default=[], help="P: Line one")
    parser.add_argument("--p2", action='append', type=float, dest='p2', default=[], help="P: Line Two")
    parser.add_argument("--p3", action='append', type=float, dest='p3', default=[], help="P: Line Three")
    parser.add_argument("--q1", action='append', type=float, dest='q1', default=[], help="Q: Line One")
    parser.add_argument("--q2", action='append', type=float, dest='q2', default=[], help="Q: Line Two")
    parser.add_argument("--q3", action='append', type=float, dest='q3', default=[], help="Q: Line Three")
    parser.add_argument("--numU", type=int, default=2, help="how many UAVs")
    parser.add_argument("--typeT", action='append', type=int, dest='typeT', default=[], help="target types")
    return parser.parse_args()


def make_env(arglist):
    from MAEnv.environment import MultiAgentEnv
    import MAEnv.scenarios as scenarios

    # load scenario from script
    scenario = scenarios.load(arglist.scenario + ".py").Scenario()

    # create world and env
    world = scenario.make_js_world(arglist.numU, arglist.typeT)
    env = MultiAgentEnv(world, scenario.reset_world, scenario.reward, scenario.observation)
    return env, world


def get_controller(env, world, arglist):
    ControllerSet = []

    for i in range(arglist.numU):
        control = []
        control.append(PM_P.PolicyMaker_Probability("agent_%d" % i, env, world, i, arglist))
        control.append(PP_S.PathPlanner_Simple("agent_%d" % i, env, world, i, arglist))
        control.append(MC_L.MotionController_L1_TECS("agent_%d" % i, env, world, i, arglist))
        control.append(IC_P.InnerController_PID("agent_%d" % i, env, world, i, arglist))
        ControllerSet.append(control)

    return ControllerSet


def update_action(obs_n, WorldTarget, step, NewController):

    action_n = []

    for i in range(arglist.numU):

        list_i = NewController[i][0]. \
            make_policy(WorldTarget, obs_n, step)

        NewController[i][6] = list_i[1][2]

        pointAi, pointBi, finishedi, NewController[i][5] = NewController[i][1].\
            planpath(list_i, obs_n[i], NewController[i][4], step)

        acctEi, acclEi, NewController[i][4] = NewController[i][2]. \
            get_expected_action(obs_n[i], pointAi, pointBi, step, finishedi)

        actioni = NewController[i][3]. \
            get_action(obs_n[i], acctEi, acclEi, step, finishedi)

        action_n.append(actioni)

    for i in range(arglist.numU):

        action = np.random.random_sample(5) * 2 - 1
        action_n.append(action)

    return action_n


def augment_view(env, world, NewController):
    for i in range(arglist.numU):
        world.agents[i].attacking_to = NewController[i][6]
        if NewController[i][5]:
            world.agents[i].attacking = True


if __name__ == '__main__':
    arglist = parse_args()
    logging.info(arglist)
    logging.info('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>')

    # Create environment
    env, world = make_env(arglist)
    WorldTarget = []
    for i, landmark in enumerate(world.targets):
        WorldTarget.append([landmark.state.p_pos[0], landmark.state.p_pos[1],
                            landmark.lon, landmark.lat, landmark.alt,
                            landmark.value, landmark.defence, landmark.type, i])

    episode = 0
    t_start = time.time()

    while episode < arglist.episode_max:

        # Create Controller (重置实例变量)
        NewController = get_controller(env, world, arglist)

        # Rest Controller (重置类变量)
        PM_P.PolicyMaker_Probability.Found_Target_Set = []
        PM_P.PolicyMaker_Probability.Found_Target_Info = []
        PM_P.PolicyMaker_Probability.Attacked_Target_Index = []
        PM_P.PolicyMaker_Probability.Remain_UAV_Set = []
        PM_P.PolicyMaker_Probability.Remain_Target_Set = []
        PM_P.PolicyMaker_Probability.Current_Target_Index = -1
        PM_P.PolicyMaker_Probability.Current_Price_Set = []
        PM_P.PolicyMaker_Probability.Current_Price_Result = []
        for i in range(arglist.numU):
            PM_P.PolicyMaker_Probability.Remain_UAV_Set.append(i)

        obs_n = env.reset()
        episode += 1
        step = 0

        while step <= arglist.step_max:

            # get action
            print('>>> step ', step)
            action_n = update_action(obs_n, WorldTarget, step, NewController)

            # environment step
            new_obs_n, rew_n, done_n, info_n = env.jstep(action_n)
            print('action_n', action_n)
            print('obs_n_diff', new_obs_n - obs_n)
            step += 1
            obs_n = new_obs_n

            # for displaying
            augment_view(env, world, NewController)
            # env.render()  # could be commented out

            # for recording
            if step == arglist.step_max:
                print('>>>>>>>>>>> Episode', episode)
                pairing = []
                for i in range(arglist.numU):
                    pairing.append(world.agents[i].attacking_to)
                print('pairing: ', pairing)
                # print('reward:', rew_n[0][0])
                timing = round(time.time() - t_start, 3)
                t_start = time.time()
                print('timing: ', timing)
                time.sleep(1)
                # if episode == 1:
                #    logging.info('EPISODE | PAIRING                                             REWARD          TIME')
                # logging.info([episode]+pairing+[rew_n[0][0]]+[timing])
                # if episode == arglist.episode_max:
                #    logging.info('\n')
