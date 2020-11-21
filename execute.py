# coding=utf-8

import argparse
import time
import MAControl.PTMA.InnerController_PID as IC_P
import MAControl.PTMA.MotionController_L1_TECS_T as MC_L
import MAControl.PTMA.PathPlanner_EdgeWaypoint as PP_S
import MAControl.PTMA.PolicyMaker_Probability as PM_P
import matplotlib.pyplot as plt
import math
import logging

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
    parser.add_argument("--episode-max", type=int, default=4, help="maximum episodes")
    parser.add_argument("--p1", action='append', type=float, dest='p1', default=[], help="P: Line one")
    parser.add_argument("--p2", action='append', type=float, dest='p2', default=[], help="P: Line Two")
    parser.add_argument("--p3", action='append', type=float, dest='p3', default=[], help="P: Line Three")
    parser.add_argument("--q1", action='append', type=float, dest='q1', default=[], help="Q: Line One")
    parser.add_argument("--q2", action='append', type=float, dest='q2', default=[], help="Q: Line Two")
    parser.add_argument("--q3", action='append', type=float, dest='q3', default=[], help="Q: Line Three")
    parser.add_argument("--numU", type=int, default=3, help="how many UAVs")
    parser.add_argument("--typeT", action='append', type=int, dest='typeT', default=[], help="target types")
    return parser.parse_args()


def make_env(arglist):
    from MAEnv.environment import MultiAgentEnv
    import MAEnv.scenarios as scenarios
    # >>> load scenario from script
    scenario = scenarios.load(arglist.scenario + ".py").Scenario()
    # >>> create world and env
    world = scenario.make_js_world(arglist.numU, arglist.typeT)
    env = MultiAgentEnv(world, scenario.reset_world, scenario.reward, scenario.observation)
    return env, world


def get_controller(env, world, arglist):
    ControllerSet = []

    for ii in range(arglist.numU):
        control = []
        control.append(PM_P.PolicyMaker_Probability("agent_%d" % ii, env, world, ii, arglist))
        control.append(PP_S.PathPlanner_EdgeWaypoint("agent_%d" % ii, env, world, ii, arglist))
        control.append(MC_L.MotionController_L1_TECS("agent_%d" % ii, env, world, ii, arglist))
        control.append(IC_P.InnerController_PID("agent_%d" % ii, env, world, ii, arglist))
        control.append([False, False, -1, 0])   # FLAG-SET
        # Arrived # Isattacking # Which-Target(-Under-Attack-) # UAV-State
        ControllerSet.append(control)

    return ControllerSet


def update_action(obs_n, WorldTarget, step, Controller):

    action_n = []

    for i in range(arglist.numU):

        para_list = Controller[i][0].make_policy(WorldTarget, obs_n, step)

        Controller[i][4][2] = para_list[1][0]

        pointAi, pointBi, finishedi, Controller[i][4][3] = Controller[i][1].\
            planpath(para_list, obs_n[i], Controller[i][4][0], step)

        print('i-A-B', i, pointAi, pointBi)

        pitch_sp, thr_sp, roll_sp, nav_bearing = Controller[i][2]. \
            get_expected_action(obs_n[i], pointAi, pointBi, step, finishedi)

        actioni = Controller[i][3]. \
            get_action(obs_n[i], pitch_sp, thr_sp, roll_sp, nav_bearing, step, finishedi)

        action_n.append(actioni)

    return action_n


def augment_view(wOrld, CONtroller):
    for ii in range(arglist.numU):
        wOrld.agents[ii].attacking_to = CONtroller[ii][4][2]
        if CONtroller[ii][4][1]:
            wOrld.agents[ii].attacking = True
    return None


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

    # START
    episode = 0
    t_start = time.time()

    while episode < arglist.episode_max:

        # Create Controller (重置实例变量)
        MainController = get_controller(env, world, arglist)

        # Rest Controller (重置类变量)
        PM_P.PolicyMaker_Probability.SEEN_TARGETS = []
        PM_P.PolicyMaker_Probability.RESULT = []
        PM_P.PolicyMaker_Probability.Prices = []
        PM_P.PolicyMaker_Probability.Occupied_U = []
        PM_P.PolicyMaker_Probability.Attacked_T = []

        # start
        obs_n = env.reset()
        episode += 1
        step = 0
        # pos #
        lon = [[] for i in range(arglist.numU)]
        lat = [[] for i in range(arglist.numU)]
        alt = [[] for i in range(arglist.numU)]
        # command #
        ele = [[] for i in range(arglist.numU)]
        ail_r = [[] for i in range(arglist.numU)]
        thr = [[] for i in range(arglist.numU)]
        # state #
        pitch = [[] for i in range(arglist.numU)]
        roll = [[] for i in range(arglist.numU)]
        vel = [[] for i in range(arglist.numU)]
        sstep = []
        # plot #
        fig = plt.figure()
        ax1 = fig.add_subplot(221)
        ax2 = fig.add_subplot(222)
        ax3 = fig.add_subplot(223)
        ax4 = fig.add_subplot(224)
        plt.ion()

        while step <= arglist.step_max:

            # get action
            print('>>> step ', step)
            action_n = update_action(obs_n, WorldTarget, step, MainController)

            # environment step
            new_obs_n, rew_n, done_n, info_n = env.jstep(action_n)
            # print('action_n', action_n)
            # print('obs_n_diff', new_obs_n - obs_n)
            step += 1
            obs_n = new_obs_n

            # for displaying
            augment_view(world, MainController)
            env.render()  # could be commented out

            # IN/OUT DATA OF CONTROLLERS
            sstep.append(step)
            for i in range(arglist.numU):
                lon[i].append(obs_n[i][16])
                lat[i].append(obs_n[i][15])
                alt[i].append(obs_n[i][0])
                ele[i].append(action_n[i][2])
                ail_r[i].append(action_n[i][1])
                thr[i].append(action_n[i][4])
                pitch[i].append(obs_n[i][1]/math.pi*180)
                roll[i].append(obs_n[i][2]/math.pi*180)
                vel[i].append(math.sqrt(obs_n[i][4]**2+obs_n[i][5]**2+obs_n[i][6]**2))
            # Real-time plotting (matplotlib) for visualized DEBUG
            for i in range(arglist.numU):
                ax1.plot(sstep, alt[i], color='c')
                ax2.plot(sstep, roll[i], color='r')
                ax2.plot(sstep, pitch[i], color='b')
                ax3.plot(sstep, vel[i], color='y')
                ax4.plot(sstep, ele[i], color='k')
                ax4.plot(sstep, ail_r[i])
                ax4.plot(sstep, thr[i], color='g')
            plt.show()
            plt.pause(0.0001)  # 暂停一瞬

            # for recording
            if step == arglist.step_max:
                print('>>>>>>>>>>> Episode', episode)
                pairing = []
                for i in range(arglist.numU):
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
