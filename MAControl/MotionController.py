import numpy as np
from sympy import *
import os


class MotionController:
    def __init__(self, env, world, arglist):
        print("L1 control init")
        self.env = env
        self.world = world
        self.arglist = arglist
        self.obs_shape_n = [env.observation_space[i].shape for i in range(env.n)]
        self.actions_n = [world.dim_p * 2 + 1 for i in range(env.n)]
        self.n_features = self.obs_shape_n[0][0]
        self.pointA = (0, 0)
        self.pointB = (0, 0)

        self.build_net()

        t_params = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope='target_net')
        e_params = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope='eval_net')

        with tf.variable_scope('hard_replacement'):
            self.target_replace_op = [tf.assign(t, e) for t, e in zip(t_params, e_params)]

        self.sess = tf.Session()
        self.sess.run(tf.global_variables_initializer())
        self.cost_his = []
        self.saver = tf.train.Saver(max_to_keep=2)
        if self.arglist.load_dir != "":
            self.load_model(self.arglist.load_dir)