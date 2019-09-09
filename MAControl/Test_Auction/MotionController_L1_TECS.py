from MAControl.Base.MotionController import MotionController


class MotionController_L1_TECS(MotionController):

    def __init__(self, name, env, world, agent_index, arglist):
        super(MotionController_L1_TECS, self).__init__(name, env, world, agent_index, arglist)
        # extra params
        pass

    def get_expected_action(self, obs, pointAi, pointBi, step):
        # actual code
        pass

