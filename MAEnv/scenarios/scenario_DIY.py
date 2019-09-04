# 环境长度 1 = 实际长度 1000 米 = 1 千米

import numpy as np
from MAEnv.core import World, Agent, Landmark
from MAEnv.scenario import BaseScenario


class Scenario(BaseScenario):
    def make_world(self):
        world = World()
        # set any world properties first
        num_agents = 3
        num_landmarks = 4
        world.damping = 0  # 取消第一种阻尼计算方式
        world.damping2 = 10  # 调整第二种阻尼计算方式的参数
        # add agents
        world.agents = [Agent() for i in range(num_agents)]
        for i, agent in enumerate(world.agents):
            agent.name = 'agent %d' % i
            agent.collide = True
            agent.silent = True
            agent.size = 0.01  # 10米
        # add landmarks
        world.landmarks = [Landmark() for i in range(num_landmarks)]
        for i, landmark in enumerate(world.landmarks):
            landmark.name = 'landmark %d' % i
            landmark.collide = False
            landmark.movable = False
            landmark.size = 0.01  # 10米
        # make initial conditions
        self.reset_world(world)
        return world

    def reset_world(self, world):

        for i, agent in enumerate(world.agents):
            agent.state.p_pos = np.random.uniform(-0.9, -0.8, world.dim_p)
            agent.state.p_vel = np.array([0, 0.05])  # 50 米/秒
            agent.state.p_acc = np.array([0, 0])
            agent.color = np.array([0.47, 0.79, 0.79])

        for i, landmark in enumerate(world.landmarks):
            landmark.state.p_vel = np.zeros(world.dim_p)
            landmark.color = np.random.uniform(0, 1, 3)

        rangee = 0.9
        world.landmarks[0].state.p_pos = np.array([-rangee, -rangee])
        world.landmarks[1].state.p_pos = np.array([-rangee, +rangee])
        world.landmarks[2].state.p_pos = np.array([+rangee, +rangee])
        world.landmarks[3].state.p_pos = np.array([+rangee, -rangee])


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

