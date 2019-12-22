import tensorflow as tf
import numpy as np
import os


class DQN_trainer():
    def __init__(self,
                 env,
                 world,
                 arglist,
                 n_features,
                 n_actions):

        print("DQN_trainer init")

        self.env = env
        self.world = world
        self.n_actions = n_actions
        self.n_features = n_features
        self.lr = arglist.lr
        self.gamma = arglist.gamma
        self.batch_size = arglist.batch_size
        self.memory_size = arglist.memory_size
        self.replace_target_iter = arglist.replace_target_iter
        self.memory = np.zeros((self.memory_size, self.n_features * 2 + 1 + 1))  # 1 + 1 -> act + reward
        self.epsilon = arglist.e_greedy
        self.epsilon_max = arglist.e_greedy_max
        self.epsilon_increment = arglist.e_greedy_increment
        self.use_doubleQ = arglist.use_doubleQ
        self.load_dir = arglist.load_dir
        self.save_dir = arglist.save_dir
        self.save_rate = arglist.save_rate
        self.learn_step_counter = 0

        self.s = None
        self.s_ = None
        self.r = None
        self.a = None
        self.q_eval = None
        self.q_next = None
        self.q_target = None
        self.q_eval4next = None
        self.q_eval_wrt_a = None
        self.loss = None
        self._train_op = None
        self.build_net()

        t_params = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope='target_net')
        e_params = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope='eval_net')
        en_params = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope='eval_net4next')

        # 将评估网络的参数赋值给目标网络
        with tf.variable_scope('hard_replacement'):
            self.target_replace_op = [tf.assign(t, e) for t, e in zip(t_params, e_params)]

        with tf.variable_scope("hard_replacement2"):
            self.target_replace_op2 = [tf.assign(t, e) for t, e in zip(en_params, e_params)]

        self.sess = tf.Session()
        self.sess.run(tf.global_variables_initializer())
        self.cost_his = list()
        self.saver = tf.train.Saver(max_to_keep=1)
        if self.load_dir != "":
            self.load_model(self.load_dir)

    def build_net(self):

        print("building net")
        self.s = tf.placeholder(tf.float32, [None, self.n_features], name='s')
        self.s_ = tf.placeholder(tf.float32, [None, self.n_features], name='s_')
        self.r = tf.placeholder(tf.float32, [None, ], name='r')  # input Reward
        self.a = tf.placeholder(tf.int32, [None, ], name='a')  # input Action

        w_initializer, b_initializer = tf.random_normal_initializer(0., 0.3), tf.constant_initializer(0.1)

        # ------------------ build evaluate_net ------------------
        with tf.variable_scope('eval_net'):
            e1 = tf.layers.dense(self.s, 20, tf.nn.relu, kernel_initializer=w_initializer,
                                 bias_initializer=b_initializer, name='e1')
            self.q_eval = tf.layers.dense(e1, self.n_actions, kernel_initializer=w_initializer,
                                          bias_initializer=b_initializer, name='q')

        # ------------------ build target_net --------------------
        with tf.variable_scope('target_net'):
            t1 = tf.layers.dense(self.s_, 20, tf.nn.relu, kernel_initializer=w_initializer,
                                 bias_initializer=b_initializer, name='t1')
            self.q_next = tf.layers.dense(t1, self.n_actions, kernel_initializer=w_initializer,
                                          bias_initializer=b_initializer, name='t2')

        # ------------------ build eval_net4next --------------------
        with tf.variable_scope('eval_net4next'):
            t1 = tf.layers.dense(self.s_, 20, tf.nn.relu, kernel_initializer=w_initializer,
                                 bias_initializer=b_initializer, name='ev1')
            self.q_eval4next = tf.layers.dense(t1, self.n_actions, kernel_initializer=w_initializer,
                                               bias_initializer=b_initializer, name='ev2')

        # --------------------- net update -----------------------
        if self.use_doubleQ:
            value_i = tf.to_int32(tf.argmax(self.q_eval4next, axis=1))
            range_i = tf.range(tf.shape(self.a)[0], dtype=tf.int32)
            index_a = tf.stack([range_i, value_i], axis=1)
            max_q = tf.gather_nd(params=self.q_next, indices=index_a)
        else:
            max_q = tf.reduce_max(self.q_next, axis=1, name='Qmax_s_')  # shape=(None, )

        with tf.variable_scope('q_target'):
            q_target = self.r + self.gamma * max_q
            self.q_target = tf.stop_gradient(q_target)

        with tf.variable_scope('q_eval'):
            a_indices = tf.stack([tf.range(tf.shape(self.a)[0], dtype=tf.int32), self.a], axis=1)
            self.q_eval_wrt_a = tf.gather_nd(params=self.q_eval, indices=a_indices)  # shape=(None,)

        with tf.variable_scope('loss'):
            self.loss = tf.reduce_mean(tf.squared_difference(self.q_target, self.q_eval_wrt_a, name='TD_error'))

        with tf.variable_scope('train'):
            self._train_op = tf.train.RMSPropOptimizer(self.lr).minimize(self.loss)

    def store_transition(self, s, a, r, s_):
        if not hasattr(self, 'memory_counter'):
            self.memory_counter = 0
        transition = np.hstack((s, a, r, s_))  # 按行合并
        # replace the old memory with new memory
        index = self.memory_counter % self.memory_size
        self.memory[index, :] = transition
        self.memory_counter += 1

    def choose_action(self, observation):

        # to have batch dimension when feed into tf placeholder

        if np.random.uniform() < self.epsilon:
            # forward feed the observation and get q value for every actions
            actions_value = self.sess.run(self.q_eval, feed_dict={self.s: observation})
            action = np.argmax(actions_value, axis=1)
            action = list(action)[0]

        else:
            action = np.random.randint(0, self.n_actions)

        return action

    def learn(self):

        self.sess.run(self.target_replace_op2)
        # check to replace target parameters
        if self.learn_step_counter % self.replace_target_iter == 0:
            self.sess.run(self.target_replace_op)
            print('\ntarget_params_replaced\n')

        # sample batch memory from all memory
        if self.memory_counter > self.memory_size:
            sample_index = np.random.choice(self.memory_size, size=self.batch_size)
        else:
            sample_index = np.random.choice(self.memory_counter, size=self.batch_size)
        batch_memory = self.memory[sample_index, :]

        _, cost = self.sess.run(
            [self._train_op, self.loss],
            feed_dict={
                self.s: batch_memory[:, :self.n_features],
                self.a: batch_memory[:, self.n_features],
                self.r: batch_memory[:, self.n_features + 1],
                self.s_: batch_memory[:, -self.n_features:],
            })

        self.cost_his.append(cost)

        # increasing epsilon
        self.epsilon = self.epsilon + self.epsilon_increment if self.epsilon < self.epsilon_max else self.epsilon_max
        self.learn_step_counter += 1

        if self.learn_step_counter % self.save_rate == 0:
            self.save_model(self.save_dir, self.learn_step_counter)

    def save_model(self, path, step):
        if os.path.isdir(path) is False:
            os.makedirs(path)
        self.saver.save(self.sess, save_path=path, global_step=step)

    def load_model(self, path):
        print("loading model")
        load = tf.train.latest_checkpoint(path)
        self.saver.restore(self.sess, load)
