# 环境长度 1 = 实际长度 1000 米 = 1 千米
from abc import ABC

import numpy as np
import random
from MAEnv.core import World, Landmark
from MAEnv.scenario import BaseScenario
import MAEnv.scenarios.TargetProfile as T
from Mini0jsbsim.simulation import Simulation
import Mini0jsbsim.properties as prp


class Scenario(BaseScenario, ABC):
    def make_js_world(self, agent_num, target_type):
        world = World()
        # set agents = UAVs (in air)
        num_agents = agent_num
        world.agents = [Simulation() for i in range(num_agents)]
        for i, agent in enumerate(world.agents):
            agent.name = 'agent %d' % i
            agent.action_callback = [agent.__getitem__(prp.aileron_left),
                                     agent.__getitem__(prp.aileron_right),
                                     agent.__getitem__(prp.elevator),
                                     agent.__getitem__(prp.rudder),
                                     agent.__getitem__(prp.throttle),
                                     agent.__getitem__(prp.gear)]
        # set other entities (on ground)
        num_targets = T.num_targets
        num_obstacles = 0
        num_grids = 5
        # add landmarks
        world.targets = [Landmark() for i in range(num_targets)]
        VALUE = [2, 10, 5]
        DEFENCE = [5, 1, 2]
        for i, landmark in enumerate(world.targets):
            landmark.name = 'target %d' % i
            landmark.collide = False
            landmark.movable = False
            landmark.value = VALUE[target_type[i]-1]
            landmark.size = T.target_size[i] * 0.01
            landmark.defence = DEFENCE[target_type[i]-1]
            landmark.attacking = False
            landmark.type = target_type[i]
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
            agent.reinitialise()
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
        agent.obs = [agent.__getitem__(prp.altitude_sl_ft),
                     agent.__getitem__(prp.pitch_rad), agent.__getitem__(prp.roll_rad), agent.__getitem__(prp.heading_deg),
                     agent.__getitem__(prp.u_fps), agent.__getitem__(prp.v_fps), agent.__getitem__(prp.w_fps),
                     agent.__getitem__(prp.u_aero_fps), agent.__getitem__(prp.v_aero_fps), agent.__getitem__(prp.w_aero_fps),
                     agent.__getitem__(prp.v_north_fps), agent.__getitem__(prp.v_east_fps),
                     agent.__getitem__(prp.p_radps), agent.__getitem__(prp.q_radps), agent.__getitem__(prp.r_radps),
                     agent.__getitem__(prp.lat_geod_deg), agent.__getitem__(prp.lng_geoc_deg)]
        # [0] altitude [1] pitch [2] roll [3] heading(yaw)
        # [4] u [5] v [6] w
        # [7] u-aero [8] v-aero [9] w-aero [10] v-north [11] v-east
        # [12] p [13] q [14] r
        # [15] lat [16] lon
        return agent.obs
