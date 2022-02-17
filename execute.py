# coding=utf-8

import argparse
import time
import numpy as np
import MAControl.Test_Auction.InnerController_PID as IC_P
import MAControl.Test_Auction.MotionController_L1_TECS as MC_L
import MAControl.Test_Auction.PathPlanner_Simple as PP_S
import MAControl.Test_Auction.PolicyMaker_Auction as PM_A
import MAControl.NMPC.NMPC as MPC_P
from aiframe.msg import simulationMessageList
from aiframe.msg import vectorlist
from aiframe.msg import simulationMessage


def parse_args():
    parser = argparse.ArgumentParser("Control Experiments for Multi-Agent Environments")
    parser.add_argument("--scenario", type=str, default="scenario2_Target", help="name of the scenario script")
    parser.add_argument("--step-max", type=int, default=4000, help="maximum steps")
    parser.add_argument("--T", type=int, default="4", help="predictive time")
    parser.add_argument("--dt", type=float, default="0.2", help="one step time")
    parser.add_argument("--num_agent", type=int, default="5", help="one step time")

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
        control.append(PP_S.PathPlanner_EdgeWaypoint("agent_%d" % i, env, world, i, arglist))
        control.append(MC_L.MotionController_L1_TECS("agent_%d" % i, env, world, i, arglist))
        control.append(IC_P.InnerController_PID("agent_%d" % i, env, world, i, arglist))
        control.append(False)  # Arriveflag
        control.append(False)  # Isattacking
        ControllerSet.append(control)

    return ControllerSet


def update_action(env, world, obs_n, step, NewController, NMPCCommand):

    # WorldTarget
    WorldTarget = []
    for i, landmark in enumerate(world.targets):
        WorldTarget.append([landmark.state.p_pos[0], landmark.state.p_pos[1], landmark.state.p_vel[0],
                            landmark.state.p_vel[1], landmark.value, landmark.defence])

    # get action
    action_n = []

    for i in range(env.n):

        # list_i = [1, [NMPCCommand[3*i+0], NMPCCommand[3*i+1]]]
        #
        # # list_i = NewController[i][0]. \
        # #     make_policy(WorldTarget, obs_n, step)
        #
        # pointAi, pointBi, finishedi, NewController[i][5], tmp = NewController[i][1].\
        #     planpath(list_i, obs_n[i], NewController[i][4], step)
        #
        # acctEi, acclEi, NewController[i][4] = NewController[i][2]. \
        #     get_expected_action(obs_n[i], pointAi, pointBi, step, finishedi)
        #
        # actioni = NewController[i][3]. \
        #     get_action(obs_n[i], acctEi, acclEi, step, finishedi)

        actioni = [0, NMPCCommand[3*i+0], 0, NMPCCommand[3*i+1], 0]
        action_n.append(actioni)

    return action_n


def augment_view(env, world, NewController):
    for i in range(env.n):
        if NewController[i][5]:
            world.agents[i].attacking = True


def NMPC_Init(obs_n, num_agent, arglist):
    T = arglist.T
    dt = arglist.dt
    initx = []

    for i in range(len(obs_n)):
        initx.append(obs_n[i][2])
        initx.append(obs_n[i][3])
        initx.append(-2.5)

    for i in range(len(obs_n)):
        initx.append(obs_n[i][0])
        initx.append(obs_n[i][1])
        initx.append(0)

    x_init = np.array(initx)
    x_init = np.stack(x_init)
    u_init = np.array([0] * 3 * len(obs_n))

    timeStep = [i for i in range(int(T / dt) + 1)]
    tmp = []
    for i in range(3*num_agent):
        tmp.append(timeStep)
    tmp1 = np.array(tmp)
    tmp2 = []
    tmpArray = []
    for i in range(num_agent):
        tmpArray.append(1)
        tmpArray.append(0)
        tmpArray.append(0)

    for i in range(int(T / dt) + 1):
        tmp2.append(tmpArray)
    tmp3 = 0.5 * dt * np.array(tmp2).transpose()
    c = []
    for i in range(int(T / dt) + 1):
        c.append(initx[0:3*num_agent])
    c = np.array(c).transpose()
    posTraj = c + tmp3 * tmp1
    x_traj_init = np.append(posTraj, tmp3 * 10, axis=0)

    u_traj_init = np.zeros((3*num_agent, int(T / dt)))

    # init NMPC Controller
    NMPCController = MPC_P.NMPC(num_agent, T, dt)
    NMPCController.initModel(np.array(initx))

    return NMPCController, x_traj_init, u_traj_init, x_init


if __name__ == '__main__':

    arglist = parse_args()

    # Create environment
    env, world = make_env(arglist)

    obs_n = env.reset()

    # Create Controller
    NewController = get_controller(env, world, arglist)
    NMPCController,  x_traj_init, u_traj_init, x_init = NMPC_Init(obs_n, len(obs_n), arglist)

    step = 0
    start = time.time()

    x_history = []

    while True:
        print(time.time())

        # get action
        print('>>>> step', step)

        for i in range(int(arglist.T / arglist.dt) + 1):
            NMPCController.solver.set(i, "x", x_traj_init[:, i])
        for i in range(int(arglist.T / arglist.dt)):
            NMPCController.solver.set(i, "u", u_traj_init[:, i])
        NMPCController.solver.set(0, 'lbx', x_init)
        NMPCController.solver.set(0, 'ubx', x_init)
        x_history.append(x_init.copy())
        status = NMPCController.solver.solve()
        if status == 4:
            tmp1 = np.array(x_traj_init)
            tmp1.tofile('x_traj_init.bin')
            tmp2 = np.array(u_traj_init)
            tmp2.tofile('u_traj_init.bin')
            tmp3 = np.array(x_init)
            tmp3.tofile('x_init.bin')
            print("error")

            timeStep = [i for i in range(int(arglist.T / arglist.dt) + 1)]
            tmp = []
            for i in range(3 * arglist.num_agent):
                tmp.append(timeStep)
            tmp1 = np.array(tmp)
            tmp2 = []
            tmpArray = []
            for i in range(arglist.num_agent):
                tmpArray.append(1)
                tmpArray.append(0)
                tmpArray.append(0)

            for i in range(int(arglist.T / arglist.dt) + 1):
                tmp2.append(tmpArray)
            tmp3 = 0.5 * arglist.dt * np.array(tmp2).transpose()
            c = []
            for i in range(int(arglist.T / arglist.dt) + 1):
                c.append(x_init[0:3 * len(obs_n)])
            c = np.array(c).transpose()
            posTraj = c + tmp3 * tmp1
            x_traj_init = np.append(posTraj, tmp3 * 10, axis=0)

            u_traj_init = np.zeros((3 * len(obs_n), int(arglist.T / arglist.dt)))
            for i in range(int(arglist.T / arglist.dt) + 1):
                NMPCController.solver.set(i, "x", x_traj_init[:, i])
            for i in range(int(arglist.T / arglist.dt)):
                NMPCController.solver.set(i, "u", u_traj_init[:, i])
            NMPCController.solver.set(0, 'lbx', x_init)
            NMPCController.solver.set(0, 'ubx', x_init)
            x_history.append(x_init.copy())
            status = NMPCController.solver.solve()

        uOptAcados = np.ndarray((int(arglist.T / arglist.dt), 3*len(obs_n)))
        xOptAcados = np.ndarray((int(arglist.T / arglist.dt) + 1, 6*len(obs_n)))
        for i in range(int(arglist.T / arglist.dt)):
            xOptAcados[i, :] = NMPCController.solver.get(i, "x")
            uOptAcados[i, :] = NMPCController.solver.get(i, "u")
        xOptAcados[int(arglist.T / arglist.dt), :] = NMPCController.solver.get(int(arglist.T / arglist.dt), "x")
        x_traj_init = xOptAcados[1:]
        x_traj_init = np.append(x_traj_init, [xOptAcados[int(arglist.T / arglist.dt), :]], axis=0)
        x_traj_init = x_traj_init.transpose()
        u_traj_init = uOptAcados[1:]
        u_traj_init = np.append(u_traj_init, [uOptAcados[int(arglist.T / arglist.dt)-1, :]], axis=0)
        u_traj_init = u_traj_init.transpose()

        action_n = update_action(env, world, obs_n, step, NewController, uOptAcados[0, :])
        # action_n = update_action(env, world, obs_n, step, NewController, xOptAcados[0, :])

        # environment step
        new_obs_n, rew_n, done_n, info_n = env.step(action_n)
        step += 1
        obs_n = new_obs_n

        initx = []
        for i in range(len(obs_n)):
            initx.append(obs_n[i][2])
            initx.append(obs_n[i][3])
            initx.append(-2.5)
        for i in range(len(obs_n)):
            initx.append(obs_n[i][0])
            initx.append(obs_n[i][1])
            initx.append(0)
        x_init = np.array(initx)
        x_init = np.stack(x_init)

        # for displaying
        # time.sleep(0.01)
        augment_view(env, world, NewController)
        env.render()
        # print('>>>> step', step)

        for i in range(len(world.agents)):
            if obs_n[i][3] > 8:
                exit()