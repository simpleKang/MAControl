from abc import ABC, abstractmethod


class PolicyMaker(ABC):

    def __init__(self, name, env, world, agent_index, arglist):
        pass

    @abstractmethod
    def makepolicy(self, WorldTarget, obs_n, step):
        pass
