from MAControl.Base import PolicyMaker


class PolicyMaker_Auciton(PolicyMaker.PolicyMaker):

    def __init__(self, name, env, world, agent_index, arglist):
        super(PolicyMaker_Auciton, self).__init__()
        self.name = name
        self.env = env
        self.world = world
        self.index = agent_index
        self.arglist = arglist
        pass

    def makepolicy(self, WorldTarget, obs_n, step):
        print(self.index, ' This is a policymaker.')
        pass

