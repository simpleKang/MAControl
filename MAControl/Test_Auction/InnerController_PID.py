from MAControl.Base.InnerController import InnerController


class InnerController_PID(InnerController):

    def __init__(self, name, env, world, agent_index, arglist):
        super(InnerController_PID, self).__init__(name, env, world, agent_index, arglist)
        # extra params
        pass

    def get_action(self, obs, Eacct, Eaccl, step, finishedi):
        # actual code
        pass

