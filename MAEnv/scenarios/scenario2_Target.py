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
        num_agents = 10
        num_targets = T.num_targets
        num_obstacles = 0
        num_grids = 5
        # add agents
        world.agents = [Agent() for i in range(num_agents)]
        for i, agent in enumerate(world.agents):
            agent.name = 'agent %d' % i
            # agent.collide = True
            agent.collide = False
            agent.silent = True
            agent.size = 0.01  # 10米
            agent.UAV = True
            agent.attacking = False
        # add landmarks
        world.targets = [Landmark() for i in range(num_targets)]
        for i, landmark in enumerate(world.targets):
            landmark.UAV = False
            landmark.name = 'target %d' % i
            landmark.collide = False
            landmark.movable = False
            landmark.value = T.target_value[i]
            landmark.size = T.target_size[i] * 0.01
            landmark.defence = T.target_defence[i]
            landmark.attacking = False
        world.obstacles = [Landmark() for i in range(num_obstacles)]
        for i, landmark in enumerate(world.obstacles):
            landmark.UAV = False
            landmark.name = 'obstacle %d' % i
            landmark.collide = False
            landmark.movable = False
            landmark.size = np.ceil(random.random()*10)*0.01
            landmark.attacking = False
        world.grids = [Landmark() for i in range(num_grids)]
        for i, landmark in enumerate(world.grids):
            landmark.UAV = False
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
            agent.state.p_pos = np.random.uniform(-0.9, -0.8, world.dim_p)
            agent.state.p_vel = np.array([0, 0.05])  # 50 米/秒
            agent.state.p_acc = np.array([0, 0])
            agent.color = T.agent_color

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
        dangers = 0
        occupied_landmarks = 0
        min_dists = 0
        for l in world.landmarks:
            dists = [np.sqrt(np.sum(np.square(a.state.p_pos - l.state.p_pos))) for a in world.agents]
            min_dists += min(dists)
            rew -= min(dists)
            if min(dists) < 0.1:
                occupied_landmarks += 1
        if agent.collide:
            for a in world.agents:
                if agent == a:
                    continue
                if self.is_danger(a, agent):
                    rew -= 1
                    dangers += 1
        return (rew, dangers, min_dists, occupied_landmarks)

    def is_danger(self, agent1, agent2):
        delta_pos = agent1.state.p_pos - agent2.state.p_pos
        dist = np.sqrt(np.sum(np.square(delta_pos)))
        dist_min = (agent1.size + agent2.size) * 1.5
        return True if dist < dist_min else False

    def reward(self, agent, world):
        # Agents are rewarded based on minimum agent distance to each landmark, penalized for dangers
        rew = 0
        for l in world.landmarks:
            dists = [np.sqrt(np.sum(np.square(a.state.p_pos - l.state.p_pos))) for a in world.agents]
            rew -= min(dists)
        if agent.collide:
            for a in world.agents:
                if agent == a:
                    continue
                if self.is_danger(a, agent):
                    rew -= 1
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

