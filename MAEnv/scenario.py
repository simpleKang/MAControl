import numpy as np

# defines scenario upon which the world is built
class BaseScenario(object):
    # create elements of the world
    def make_world(self):
        raise NotImplementedError()

    # create elements of the world
    def make_s_world(self, agent_num, target_type):
        raise NotImplementedError()

    # create elements of the jsbsim-theme world
    def make_js_world(self, agent_num, target_type):
        raise NotADirectoryError()

    # create initial conditions of the world
    def reset_world(self, world):
        raise NotImplementedError()
