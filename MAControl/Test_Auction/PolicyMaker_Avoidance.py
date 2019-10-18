from MAControl.Base.PolicyMaker import PolicyMaker
from MAControl.Util.PointInRec import point_in_rec
from MAControl.Util.Constrain import constrain
import random
import numpy as np
import math


class PolicyMaker_Avoidance(PolicyMaker):

    def __init__(self, name, env, world, agent_index, arglist):
        super(PolicyMaker_Avoidance, self).__init__(name, env, world, agent_index, arglist)
        self.opt_index = 0

    def find_friends_in_sight(self):


    def make_policy(self, WorldTarget, obs_n, step):





