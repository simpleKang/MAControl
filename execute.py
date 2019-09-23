# coding=utf-8

import argparse
import time
import MAControl.Test_Auction.InnerController_PID as IC_P
import MAControl.Test_Auction.MotionController_L1_TECS as MC_L
import MAControl.Test_Auction.PathPlanner_Simple as PP_S
import MAControl.Test_Auction.PolicyMaker_Auction as PM_A
import logging

logging.basicConfig(filename='/home/samantha/gitr/logs/result.log', level=logging.INFO)
logging.info('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>')
logging.info(time.strftime('%Y-%m-%d, %H:%M:%S'))
logging.info('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>')
# 需要自行指定为本地存在的绝对路径，指定名称的文件如果不存在，会自动创建
# 如果存在，不会覆盖内容，会从结尾处向后添加新内容


def parse_args():
    parser = argparse.ArgumentParser("Control Experiments for Multi-Agent Environments")
    parser.add_argument("--scenario", type=str, default="scenario2_Target", help="name of the scenario script")
    parser.add_argument("--step-max", type=int, default=3000, help="maximum steps")
    parser.add_argument("--episode-max", type=int, default=200, help="maximum episodes")
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

    for i in range(env.n):
        control = []
        control.append(PM_A.PolicyMaker_Auction("agent_%d" % i, env, world, i, arglist))
        control.append(PP_S.PathPlanner_Simple("agent_%d" % i, env, world, i, arglist))
        control.append(MC_L.MotionController_L1_TECS("agent_%d" % i, env, world, i, arglist))
        control.append(IC_P.InnerController_PID("agent_%d" % i, env, world, i, arglist))
        control.append(False)  # Arriveflag
        control.append(False)  # Isattacking
        control.append(-1)  # which target is it attacking
        ControllerSet.append(control)

    return ControllerSet


def update_action(env, world, obs_n, step, NewController):

    WorldTarget = []
    for i, landmark in enumerate(world.targets):
        WorldTarget.append([landmark.state.p_pos[0], landmark.state.p_pos[1], landmark.state.p_vel[0],
                            landmark.state.p_vel[1], landmark.value, landmark.defence, landmark.type, i])

    action_n = []
    for i in range(env.n):

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

    return action_n


def augment_view(env, world, NewController):
    for i in range(env.n):
        world.agents[i].attacking_to = NewController[i][6]
        if NewController[i][5]:
            world.agents[i].attacking = True


if __name__ == '__main__':
    arglist = parse_args()

    # Create environment
    env, world = make_env(arglist)

    episode = 0
    t_start = time.time()

    while episode < arglist.episode_max:

        # Create Controller (重置实例变量)
        NewController = get_controller(env, world, arglist)

        # Rest Controller (重置类变量)
        PM_A.PolicyMaker_Auction.Found_Target_Set = []
        PM_A.PolicyMaker_Auction.Found_Target_Info = []
        PM_A.PolicyMaker_Auction.Attacked_Target_Index = []
        PM_A.PolicyMaker_Auction.Remain_UAV_Set = []
        PM_A.PolicyMaker_Auction.Remain_Target_Set = []
        PM_A.PolicyMaker_Auction.Current_Target_Index = -1
        PM_A.PolicyMaker_Auction.Current_Price_Set = []
        PM_A.PolicyMaker_Auction.Current_Price_Result = []
        for i in range(env.n):
            PM_A.PolicyMaker_Auction.Remain_UAV_Set.append(i)

        obs_n = env.reset()
        episode += 1
        step = 0

        while step <= arglist.step_max:

            # get action
            print('>>> step ', step)
            action_n = update_action(env, world, obs_n, step, NewController)

            # environment step
            new_obs_n, rew_n, done_n, info_n = env.step(action_n)
            step += 1
            obs_n = new_obs_n

            # for displaying
            augment_view(env, world, NewController)
            # env.render()  # could be commented out

            # for recording
            if step == arglist.step_max:
                print('>>>>>>>>>>> Episode', episode)
                pairing = []
                for i in range(env.n):
                    pairing.append(world.agents[i].attacking_to)
                print('pairing: ', pairing)
                print('reward:', rew_n[0][0])
                timing = round(time.time() - t_start, 3)
                t_start = time.time()
                print('timing: ', timing)
                if episode == 1:
                    logging.info('EPISODE | PAIRING                                               REWARD          TIME')
                logging.info([episode]+pairing+[rew_n[0][0]]+[timing])
                if episode == arglist.episode_max:
                    logging.info('\n')
