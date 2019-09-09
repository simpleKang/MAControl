from MAControl.Base.PolicyMaker import PolicyMaker


class PolicyMaker_Auciton(PolicyMaker):

    def __init__(self, name, env, world, agent_index, arglist):
        super(PolicyMaker_Auciton, self).__init__(name, env, world, agent_index, arglist)
        # extra params
        pass

    def make_policy(self, WorldTarget, obs_n, step):
        # actual code
        pass

