from abc import ABC, abstractmethod


class InnerController(ABC):

    def __init__(self, name, env, world, agent_index, arglist):
        self.name = name
        self.env = env
        self.world = world
        self.index = agent_index
        self.arglist = arglist

    @abstractmethod
    def get_action(self, obs, Eacct, Eaccl, step, finishedi):
        raise NotImplementedError
