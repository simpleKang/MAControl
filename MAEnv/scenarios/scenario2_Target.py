# 环境长度 1 = 实际长度 1000 米 = 1 千米

import numpy as np
import random
from MAEnv.core import World, Agent, Landmark
from MAEnv.scenario import BaseScenario
import MAEnv.scenarios.TargetProfile as T


class Scenario(BaseScenario):
    def make_world(self):
        world = World()
        # set any world properties first
        world.damping = 0  # 取消第一种阻尼计算方式
        world.damping2 = 10  # 调整第二种阻尼计算方式的参数
        # set nums
        num_agents = 30
        num_targets = T.num_targets
        num_obstacles = 0
        num_grids = 5
        # add agents
        world.agents = [Agent() for i in range(num_agents)]
        for i, agent in enumerate(world.agents):
            agent.name = 'agent %d' % i
            agent.collide = False
            agent.silent = True
            agent.size = 0.01  # 10米
        # add landmarks
        world.targets = [Landmark() for i in range(num_targets)]
        for i, landmark in enumerate(world.targets):
            landmark.name = 'target %d' % i
            landmark.collide = False
            landmark.movable = False
            landmark.value = T.target_value[i]
            landmark.size = T.target_size[i] * 0.01
            landmark.defence = T.target_defence[i]
            landmark.attacking = False
            landmark.type = T.target_type[i]
        world.obstacles = [Landmark() for i in range(num_obstacles)]
        for i, landmark in enumerate(world.obstacles):
            landmark.name = 'obstacle %d' % i
            landmark.collide = False
            landmark.movable = False
            landmark.size = np.ceil(random.random()*10)*0.01
            landmark.attacking = False
        world.grids = [Landmark() for i in range(num_grids)]
        for i, landmark in enumerate(world.grids):
            landmark.name = 'grid %d' % i
            landmark.collide = False
            landmark.movable = False
            landmark.size = 0.005
            landmark.attacking = False
        world.landmarks = world.targets + world.obstacles + world.grids
        # make initial conditions
        self.reset_world(world)
        return world

    def reset_world(self, world):

        for i, agent in enumerate(world.agents):
            agent.state.p_pos = np.random.uniform(T.agent_pos_init[0], T.agent_pos_init[1], world.dim_p)
            agent.state.p_vel = np.array([0, 0.05])  # 50 米/秒
            agent.state.p_acc = np.array([0, 0])
            agent.color = T.agent_color
            agent.attacking = False
            agent.attacking_to = -1

        for i, landmark in enumerate(world.landmarks):
            landmark.state.p_vel = np.zeros(world.dim_p)
            if i < len(world.targets):
                landmark.color = np.random.uniform(0, 1, 3)
                landmark.state.p_pos = np.array(T.target_pos[i])
            else:
                landmark.color = T.grid_color
                landmark.state.p_pos = np.array(T.grid_pos[i-len(world.targets)])

    def benchmark_data(self, agent, world):
        # returns data for benchmarking purposes
        rew = 0
        occupied_landmarks = 0
        min_dists = 0
        for l in world.landmarks:
            dists = [np.sqrt(np.sum(np.square(a.state.p_pos - l.state.p_pos))) for a in world.agents]
            min_dists += min(dists)
            rew -= min(dists)
            if min(dists) < 0.1:
                occupied_landmarks += 1
        return rew, min_dists, occupied_landmarks

    def result(self, world):
        # TARGET-UAV分配情况
        res = []
        for a, agent in enumerate(world.agents):
            res.append(agent.attacking_to)  # 长度为UAV数量，每个元素是所攻击的目标（未攻击则为-1）
        res2 = []
        res3 = []
        for t in range(len(world.targets)):
            res2.append(res.count(t))  # 长度为TARGET数量，每个元素是分配给这个目标的UAV个数
            res3.append(world.targets[t].defence)  # 长度为TARGET数量，每个元素是这个目标需要的UAV个数
        res4 = list(np.array(res2)-np.array(res3))  # 长度为TARGET数量，每个元素是上述两项差值
        res5 = []   # 长度为TARGET数量，每个元素是根据差值计算出的奖励值
        for r in res4:
            if r >= 0:
                res5.append(1/(0.2*r+1))
            else:
                res5.append(0)
        return res5

    def reward(self, agent, world):
        # 根据TARGET-UAV分配情况，计算整体评价
        # 目前只考虑了效益，尚未考虑代价
        rew = 0
        res5 = self.result(world)
        for t, target in enumerate(world.targets):
            rew += target.value * res5[t]
        return rew

    def observation(self, agent, world):
        a1 = agent.state.p_acc[0]
        a2 = agent.state.p_acc[1]
        vel_size = np.sqrt(np.square(agent.state.p_vel[0]) + np.square(agent.state.p_vel[1]))
        vel_front_unit = agent.state.p_vel / vel_size
        vel_right_unit = np.array([agent.state.p_vel[1], -1 * agent.state.p_vel[0]]) / vel_size
        a_front = np.dot(a1, vel_front_unit) + np.dot(a2, vel_front_unit)
        a_right = np.dot(a2, vel_right_unit) + np.dot(a2, vel_right_unit)
        return np.concatenate([agent.state.p_vel] + [agent.state.p_pos] + [a_front] + [a_right])
