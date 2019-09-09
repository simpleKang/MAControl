from MAControl.Base.InnerController import InnerController
import numpy as np
import math
import os
import MAControl.util as U
import MAControl.WayPointListOperator as W
import random

class InnerController_PID(InnerController):

    def __init__(self, name, env, world, agent_index, arglist):
        super(InnerController_PID, self).__init__(name, env, world, agent_index, arglist)

        # extra params
        self.dt = world.dt
        self.inner_pace = 1
        self.ITerm = 0
        self.last_error = 0
        self.action = [0, 0, 0, 0, 0]

        pass

    def get_action(self, obs, Eacct, Eaccl, step, finishedi):
        # actual code
        pass

