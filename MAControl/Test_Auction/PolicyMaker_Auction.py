from MAControl.Base.PolicyMaker import PolicyMaker


class PolicyMaker_Auciton(PolicyMaker):

    def __init__(self, name, env, world, agent_index, arglist):
        super(PolicyMaker_Auciton, self).__init__(name, env, world, agent_index, arglist)
        self.waypoint_list = []
        self.cycle_index = 100
        pass

    def makepolicy(self, WorldTarget, obs_n, step):
        print(self.index, self.cycle_index, ' This is a policymaker.')

        pass

