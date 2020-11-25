from abc import ABC, abstractmethod


class PolicyMaker(ABC):

    def __init__(self, name, env, world, agent_index, arglist):
        self.name = name
        self.env = env
        self.world = world
        self.index = agent_index
        self.arglist = arglist

    @abstractmethod
    def make_policy(self, WorldTarget, obs_n, step):
        raise NotImplementedError
