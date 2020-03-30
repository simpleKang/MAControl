# 环境长度 1 = 实际长度 1000 米 = 1 千米
# 初步应用了 entity = agent + landmark 和 agent = uav + target 的区分，删去了许多参数，仍需进一步修改
# （landmark = grid + obstacle + ...  , target = fixed_target + movable_target)

import numpy as np
import random
import math
from MAEnv.core import World, Agent, Landmark
from MAEnv.scenario import BaseScenario
import MAEnv.scenarios.TargetProfile as T


class Scenario(BaseScenario):
    def make_World(self, _uav_num):
        world = World()
        # set any world properties first
        world.damping = 0     # 取消第一种阻尼计算方式
        world.damping2 = 10   # 调整第二种阻尼计算方式的参数
        world.edge = T.edge   # 确定边界
        # set nums
        num_uav = _uav_num
        num_targets = T.num_targets
        num_grids = T.num_grids
        num_square = T.num_square

        # add agents (uavs)
        world.U_agents = [Agent() for i in range(num_uav)]
        for i, uav in enumerate(world.U_agents):
            uav.name = 'uav %d' % i
            uav.size = 0.01  # 10米
            uav.movable = True
            uav.UAV = True
            uav.H = T.UAV_H
            uav.Dam = T.UAV_Dam
            uav.w = T.UAV_w

        # add agents (targets)
        world.T_agents = [Agent() for i in range(num_targets)]
        for i, target in enumerate(world.T_agents):
            target.name = 'target %d' % i
            target.size = T.target_size[i] * 0.01
            if not T.target_movable[i]:
                target.movable = False
            target.Target = True
            target.H = T.target_H[i]
            target.Dam = T.target_Dam[i]
            target.w = T.target_w[i]

        # agents summary
        world.agents = world.U_agents + world.T_agents

        # add grids
        world.grids = [Landmark() for i in range(num_grids)]
        for i, grid in enumerate(world.grids):
            grid.name = 'grid %d' % i
            grid.size = T.grid_size
            grid.Landmark = True
            grid.obstacle = False

        # add squares
        world.squares = [Landmark() for i in range(num_square)]
        for i, square in enumerate(world.squares):
            square.name = 'square %d' % i
            square.size = T.square_size
            square.Landmark = True
            square.obstacle = True

        # landmarks summary
        world.landmarks = world.grids + world.squares

        # make initial conditions
        self.reset_world(world)
        return world

    def reset_world(self, world):

        uav_count = 0
        for i, agent in enumerate(world.agents):
            if agent.UAV:
                agent.state.p_pos = np.random.uniform(-1.6, -1.9, world.dim_p)
                # agent.state.p_pos = np.array([-0.5+i*0.5, 0])
                # agent.state.p_pos = np.array([-0.5, -0.5])
                agent.state.p_vel = np.random.uniform(-0.05, 0.05, world.dim_p)  # 50 米/秒
                # agent.state.p_vel = np.array([0.01, 0.01])
                agent.state.p_acc = np.array([0, 0])
                agent.color = T.UAV_color
                uav_count += 1
            else:
                agent.state.p_pos = np.array(T.target_pos[i-uav_count])
                agent.state.p_vel = np.random.uniform(-0.02, 0.02, world.dim_p)  # 20 米/秒
                agent.state.p_acc = np.array([0, 0])
                if agent.movable:
                    agent.color = T.movable_target_color
                else:
                    agent.color = T.fixed_target_color

        for i, landmark in enumerate(world.landmarks):
            landmark.state.p_vel = np.zeros(world.dim_p)
            if 'grid' in landmark.name:
                landmark.color = T.grid_color
                landmark.state.p_pos = T.grid_pos[i]
            elif 'square' in landmark.name:
                landmark.color = T.square_color
                landmark.state.p_pos = T.square_pos[i-T.num_grids]
            else:
                pass

    def benchmark_data(self, agent, world):
        # returns data for benchmarking purposes
        rew = agent.color[0] + world.edge
        return rew

    def reward(self, agent, world):
        rew = agent.color[0] + world.edge
        return rew

    def limited_view(self, agent, world):
        # 视场参数
        sensor_range1 = 0.0  # km
        sensor_range2 = 0.5  # km
        sensor_angle = 120 /2 /180*math.pi  # 半个视场角

        # 描述自己
        selfpos = agent.state.p_pos
        selfvel = agent.state.p_vel
        selfdir = math.atan2(selfvel[1], selfvel[0])

        # 使用极坐标描述其它个体
        bearings = []
        for i, other in enumerate(world.agents):
            if other is not agent:
                relative_pos = other.state.p_pos - selfpos
                relative_dis = np.linalg.norm(relative_pos)
                relative_bearing = math.atan2(relative_pos[1], relative_pos[0])
                if sensor_range1 <= relative_dis <= sensor_range2 and abs(relative_bearing - selfdir) <= sensor_angle:
                    bearings.append(relative_bearing)
                    bearings.append(i)
                else:
                    pass
            else:
                pass
        return bearings

    def observation(self, agent, world):
        a1 = agent.state.p_acc[0]
        a2 = agent.state.p_acc[1]
        vel_size = np.sqrt(np.square(agent.state.p_vel[0]) + np.square(agent.state.p_vel[1]))
        vel_front_unit = agent.state.p_vel / vel_size
        vel_right_unit = np.array([agent.state.p_vel[1], -1 * agent.state.p_vel[0]]) / vel_size
        a_front = np.dot([a1, 0], vel_front_unit) + np.dot([0, a2], vel_front_unit)
        a_right = np.dot([a1, 0], vel_right_unit) + np.dot([0, a2], vel_right_unit)
        bearings = self.limited_view(agent, world)
        # temp = np.concatenate([agent.state.p_vel] + [agent.state.p_pos] + [[a_front]] + [[a_right]] + [bearings])
        # print(temp)
        return np.concatenate([agent.state.p_vel] + [agent.state.p_pos] + [[a_front]] + [[a_right]] + [bearings])
