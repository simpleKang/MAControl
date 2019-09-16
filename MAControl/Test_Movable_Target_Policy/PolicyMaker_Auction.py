from MAControl.Base.PolicyMaker import PolicyMaker


class PolicyMaker_Target(PolicyMaker):

    def __init__(self, name, env, world, agent_index, arglist):
        super(PolicyMaker_Target, self).__init__(name, env, world, agent_index, arglist)
        self.opt_index = 0

    def make_policy(self, WorldTarget, obs_n, step):

        return [self.opt_index, []]
