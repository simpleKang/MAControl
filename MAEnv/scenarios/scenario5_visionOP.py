# 环境长度 1 = 实际长度 1000 米 = 1 千米
# 初步应用了 entity = agent + landmark 和 agent = uav + target 的区分，删去了许多参数，仍需进一步修改
# (landmark = grid + obstacle + ...  , target = fixed_target + movable_target)
# 用来算的大小 vs 拿来看的大小 # 这个关系要理顺 视觉效果应该ok
# Require >=8 entities in the scenario for the codes to properly work -?
# Entites could NOT occupy the same space physically -?

import numpy as np
import portion as por
import os
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
        num_square = T.num_square  # zero for now

        # add agents (uavs)
        world.U_agents = [Agent() for i in range(num_uav)]
        for i, uav in enumerate(world.U_agents):
            uav.name = 'uav %d' % i
            uav.state.size = T.UAV_size * 0.01  # 10米
            uav.movable = True
            uav.UAV = True
            uav.H = T.UAV_H
            uav.Dam = T.UAV_Dam
            uav.w = T.UAV_w
            uav.rule = 'default'

        # add agents (targets)
        world.T_agents = [Agent() for i in range(num_targets)]
        for i, target in enumerate(world.T_agents):
            target.name = 'target %d' % i
            target.state.size = T.target_size[i] * 0.01
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
            grid.state.size = T.grid_size
            grid.Landmark = True
            grid.obstacle = False

        # add squares
        world.squares = [Landmark() for i in range(num_square)]
        for i, square in enumerate(world.squares):
            square.name = 'square %d' % i
            square.state.size = T.square_size
            square.Landmark = True
            square.obstacle = True

        # landmarks summary
        world.landmarks = world.grids + world.squares

        # write-o
        curdir_ = os.path.dirname(__file__)  # current directory
        pardir_ = os.path.dirname(os.path.dirname(curdir_))  # parent parent directory
        para = np.loadtxt(pardir_ + '/track/para.txt')  # currently [10,3000]
        # self.collect = int(para[2])  # so ... wtf?

        # make initial conditions
        self.reset_world(world)
        return world

    def reset_world(self, world):

        uav_count = 0
        for i, agent in enumerate(world.agents):
            if agent.UAV:
                # agent.state.p_pos = np.array([-1.2, -0.4+i*0.2])
                # agent.state.p_pos = np.array([-1.3, 0.])
                agent.state.p_pos = np.random.uniform(-1.0, 1.0, world.dim_p)

                # agent.state.p_vel = np.array([0.01, 0.00])  # 10 米/秒
                agent.state.p_vel = np.random.uniform(-0.01, 0.01, world.dim_p)

                # agent.state.p_pos, agent.state.p_vel = T.init_state.get_state(self.collect, uav_count)

                agent.state.p_acc = np.array([0, 0])
                agent.color = T.UAV_color
                uav_count += 1
            else:
                agent.state.p_pos = np.array(T.target_pos[i-uav_count])
                agent.state.p_vel = np.array([0.01, 0.00])  # 10 米/秒
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

    def retina(self, agent, world):
        # 描述自己
        selfpos = agent.state.p_pos
        selfvel = agent.state.p_vel
        self_ornt = math.atan2(selfvel[1], selfvel[0])  # orientation = ORNT

        # 视场参数
        gamma = T.blind_angle[0]  # 整个盲区角
        G1 = math.fmod(self_ornt + math.pi - gamma/2, math.pi)  # so that -math.pi <= G1 <= math.pi
        G2 = math.fmod(G1 + gamma, math.pi)

        # 根据个体的方位 + 距离 + 大小 得到个体的投影角(1,2) + 距离
        # ## # entity size, bearing, distance -> entity projected-angles, distance
        retina = []  # each item = 3-tuple element
        for i, other in enumerate(world.agents):
            if other is not agent:
                relative_pos = other.state.p_pos - selfpos
                relative_dis = np.linalg.norm(relative_pos)
                relative_bearing = math.atan2(relative_pos[1], relative_pos[0])
                half_ang = math.atan2(other.state.size/2, relative_dis)
                r1 = math.fmod(relative_bearing - half_ang, math.pi)
                r2 = math.fmod(relative_bearing + half_ang, math.pi)
                r3 = relative_dis
                # ## # ↓↓ credit: KSB ↓↓ # ## #
                if abs(math.fmod(r1-G1, math.pi)) + abs(math.fmod(r1-G2, math.pi)) != gamma \
                        and abs(math.fmod(r2-G1, math.pi)) + abs(math.fmod(r2-G2, math.pi)) != gamma:
                    retina.append([r1, r2, r3])
                elif abs(math.fmod(r1-G1, math.pi)) + abs(math.fmod(r1-G2, math.pi)) == gamma \
                        and abs(math.fmod(r2-G1, math.pi)) + abs(math.fmod(r2-G2, math.pi)) == gamma:
                    retina.append([G1, G2, float('inf')])
                elif abs(math.fmod(r1-G1, math.pi)) + abs(math.fmod(r1-G2, math.pi)) == gamma:
                    retina.append([G2, r2, r3])
                else:
                    retina.append([r1, G1, r3])

        # output
        distance = [item[2] for item in retina]
        rank = [index for index, value in sorted(list(enumerate(distance)), key=lambda x: x[1])]
        return [retina, gamma, G2, rank]

    def neighbouring_view(self, agent, world):
        _retina = self.retina(agent, world)[0]
        _rank = self.retina(agent, world)[3]

        neighborhood = []
        cover = por.empty()

        for i in range(len(_rank)):
            print(i, cover, neighborhood)
            if len(neighborhood) < 7:
                # ## # get the ptem. note that [-pi] = [pi] so split there if needed
                index = _rank[i]
                item = _retina[index]
                if item[0] < item[1]:
                    ptem = por.closed(item[0], item[1])
                else:
                    ptem = por.closed(item[0], math.pi) | por.closed(-math.pi, item[1])
                # ## # if approved for inclusion
                if not (cover | ptem).difference(cover).empty:
                    neighborhood.append(item)
                    cover = cover | ptem
                    if 'uav' in world.agents[index].name:
                        othervel = world.agents[index].state.p_vel
                        other_ornt = math.atan2(othervel[1], othervel[0])  # orientation = ORNT
                        neighborhood[-1].append(1)
                        neighborhood[-1].append(other_ornt)
                    else:
                        neighborhood[-1].append(2)
                        neighborhood.append(float('nan'))
                # ## # if not
                else:
                    pass
                # ## # end
            else:
                pass

        bb = 7 - len(neighborhood)
        for i in range(bb):
            neighborhood.append([float('nan'), float('nan'), float('nan'), float('nan'), float('nan')])
        return neighborhood

    def projected_view(self, agent, world):
        num = T.num_section[0]
        retina = self.retina(agent, world)[0]
        gamma = self.retina(agent, world)[1]
        G2 = self.retina(agent, world)[2]
        rank = self.retina(agent, world)[3]
        width = (2*math.pi-gamma)/num

        projection = [0 for i in range(num)]

        for i in range(num):
            s1 = math.fmod(G2 + i*width, math.pi)
            s2 = math.fmod(G2 + (i+1)*width, math.pi)
            SS = por.open(s1, s2) if s1 < s2 else por.open(s1, math.pi) | por.open(-math.pi, s2)
            A1 = A2 = por.empty()
            a1 = a2 = 0
            # ## # from closer to further, terminate when a1 > 0.5 width or a2 > 0.5 width
            for k in rank:
                if a1/width <= 0.5 and a2/width <= 0.5:
                    item = retina[k]
                    ptem = por.open(item[0], item[1]) if item[0] < item[1] else \
                        por.open(item[0], math.pi) | por.open(-math.pi, item[1])
                    if (ptem & SS).empty:
                        pass
                    else:
                        new = (A1 | A2 | (ptem & SS)).difference(A1 | A2)
                        # add to A1 (or A2) then compute a1 (or a2) depending on what kind of entity you are
                        if 'uav' in world.agents[k].name:
                            A1 = A1 | new
                            L = len(A1)
                            a1_list = [A1[m].upper-A1[m].lower for m in range(L)]
                            a1 = sum(a1_list)
                        else:
                            A2 = A2 | new
                            L = len(A2)
                            a2_list = [A2[m].upper - A2[m].lower for m in range(L)]
                            a2 = sum(a2_list)
                elif a1/width > 0.5:
                    projection[i] = 1
                else:
                    projection[i] = 2

        return projection

    def observation(self, agent, world):
        a1 = agent.state.p_acc[0]
        a2 = agent.state.p_acc[1]
        vel_size = np.sqrt(np.square(agent.state.p_vel[0]) + np.square(agent.state.p_vel[1]))
        vel_front_unit = agent.state.p_vel / vel_size
        vel_right_unit = np.array([agent.state.p_vel[1], -1 * agent.state.p_vel[0]]) / vel_size
        a_front = np.dot([a1, 0], vel_front_unit) + np.dot([0, a2], vel_front_unit)
        a_right = np.dot([a1, 0], vel_right_unit) + np.dot([0, a2], vel_right_unit)
        n_view = self.neighbouring_view(agent, world)
        p_view = self.projected_view(agent, world)
        return np.concatenate([agent.state.p_vel] + [agent.state.p_pos] + [[a_front]] + [[a_right]] + [n_view]+[p_view])
