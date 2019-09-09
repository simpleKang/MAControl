from MAControl.Base.MotionController import MotionController
import numpy as np
import math
import os
import MAControl.util as U
import MAControl.WayPointListOperator as W
import random

class MotionController_L1_TECS(MotionController):

    def __init__(self, name, env, world, agent_index, arglist):
        super(MotionController_L1_TECS, self).__init__(name, env, world, agent_index, arglist)

        # extra params
        self.motion_pace = 5
        self.STE_rate_error = 0
        self.throttle_integ_s = 0
        self.arrive_flag = False

        pass

    def get_expected_action(self, obs, pointAi, pointBi, step):
        # actual code
        pass

