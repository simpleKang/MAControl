import numpy as np


# [w1, w2, w3, w4]
action_dict = {"0": [1., 0., 0., 0.],
               "1": [0., 1., 0., 0.],
               "2": [0., 0., 1., 0.],
               "3": [0., 0., 0., 1.]}


def net_choose_action_w(arglist, WorldTarget, obs_n, step, NewController, trainer):

    w_env = list()
    w = list()
    v_set = list()
    action_n = list()

    for i in range(arglist.agent_num):

        list_i = NewController[i][0]. \
            make_policy(WorldTarget, obs_n, step)

        v_ = list_i[1].copy()

        input_obs = obs_n[i].copy()

        for v1_4 in v_:
            input_obs = np.concatenate([input_obs] + [v1_4])

        # 在这个位置更改状态
        input_obs = np.delete(input_obs, [0, 1, 4, 5, 6, 7], axis=0)

        v_set.append(v_)

        w.append(trainer.choose_action(input_obs.reshape(1, 10)))

        w_env.append(action_dict[str(int(w[i]))])

        list_i = update_w(list_i, w_env[i])

        pointAi, pointBi, finishedi, NewController[i][5], WorldTarget = NewController[i][1]. \
            planpath(list_i, obs_n[i], NewController[i][4], step, WorldTarget)

        acctEi, acclEi, NewController[i][4] = NewController[i][2]. \
            get_expected_action(obs_n[i], pointAi, pointBi, step, finishedi)

        actioni = NewController[i][3]. \
            get_action(obs_n[i], acctEi, acclEi, step, finishedi)

        action_n.append(actioni)

    return action_n, WorldTarget, v_set, w


def update_w(list_i, w):

    if len(list_i[1]) == len(w):
        v = 0
        for i in range(len(w)):
            v += list_i[1][i] * w[i]
        list_i[1] = v

    return list_i
