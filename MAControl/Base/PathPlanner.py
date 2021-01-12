from abc import ABC, abstractmethod


class PathPlanner(ABC):

    def __init__(self, name, env, world, agent_index, arglist):
        self.name = name
        self.env = env
        self.world = world
        self.index = agent_index
        self.arglist = arglist

    @abstractmethod
    def planpath(self, para_list, obs, arrive_flag, step):
        raise NotImplementedError
