from abc import ABC, abstractmethod


class MotionController(ABC):

    def __init__(self, name, env, world, agent_index, arglist):
        self.name = name
        self.env = env
        self.world = world
        self.index = agent_index
        self.arglist = arglist

    @abstractmethod
    def get_expected_action(self, obs, pointAi, pointBi, step, finishedi):
        raise NotImplementedError

