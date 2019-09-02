# 环境长度 1 = 实际长度 1000 米 = 1 千米

import numpy as np
from MAEnv.core import World, Agent, Landmark
from MAEnv.scenario import BaseScenario


class Scenario(BaseScenario):
    def make_world(self):
        world = World()
        # set any world properties first
        num_agents = 1
        num_landmarks = 4
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

        world.agents[0].color = np.array([0.47, 0.79, 0.79])
        # world.agents[1].color = np.array([0.10, 0.20, 0.17])
        # world.agents[2].color = np.array([0.75, 0.25, 0.25])

        for i, landmark in enumerate(world.landmarks):
            landmark.state.p_vel = np.zeros(world.dim_p)

        world.landmarks[0].state.p_pos = np.array([-0.9, -0.9])
        world.landmarks[1].state.p_pos = np.array([-0.9, 0.9])
        world.landmarks[2].state.p_pos = np.array([0.9, 0.9])
        world.landmarks[3].state.p_pos = np.array([0.9, -0.9])
        # world.landmarks[4].state.p_pos = np.array([-0.6, -0.6])
        # world.landmarks[5].state.p_pos = np.array([-0.6, 0.6])
        # world.landmarks[6].state.p_pos = np.array([0.6, 0.6])
        # world.landmarks[7].state.p_pos = np.array([0.6, -0.6])
        # world.landmarks[8].state.p_pos = np.array([-0.3, -0.3])
        # world.landmarks[9].state.p_pos = np.array([-0.3, 0.3])
        # world.landmarks[10].state.p_pos = np.array([0.3, 0.3])
        # world.landmarks[11].state.p_pos = np.array([0.3, -0.3])
        world.landmarks[0].color = world.agents[0].color
        world.landmarks[1].color = world.agents[0].color
        world.landmarks[2].color = world.agents[0].color
        world.landmarks[3].color = world.agents[0].color
        # world.landmarks[4].color = world.agents[1].color
        # world.landmarks[5].color = world.agents[1].color
        # world.landmarks[6].color = world.agents[1].color
        # world.landmarks[7].color = world.agents[1].color
        # world.landmarks[8].color = world.agents[2].color
        # world.landmarks[9].color = world.agents[2].color
        # world.landmarks[10].color = world.agents[2].color
        # world.landmarks[11].color = world.agents[2].color

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
        # get positions of all entities in this agent's reference frame
        entity_pos = []
        for entity in world.landmarks:  # world.entities:
            entity_pos.append(entity.state.p_pos - agent.state.p_pos)
        # communication of all other agents
        other_pos = []
        for other in world.agents:
            if other is agent: continue
            other_pos.append(other.state.p_pos - agent.state.p_pos)
        return np.concatenate([agent.state.p_vel] + [agent.state.p_pos] + entity_pos + other_pos)
