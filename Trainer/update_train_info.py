import MAControl.Util.OnlineCoverRate as CR
import numpy as np
import math


# [UP or DOWN, RIGHT or LEFT]
action_dict = {"0": [0., 1.],    # u
               "1": [1., 1.],    # u r
               "2": [1., 0.],    #   r
               "3": [1., -1.],   # d r
               "4": [0., -1.],   # d
               "5": [-1., -1.],  # d l
               "6": [-1., 0.],   #   l
               "7": [-1., 1.]}   # u l


def update_next_state(arglist, WorldTarget, obs_n, step, NewController, trainer):

    action_n = list()
    store_state = list()

    for i in range(arglist.agent_num):

        list_i, sight_friends = NewController[i][0]. \
            make_policy(WorldTarget, obs_n, step, trainer)

        pointAi, pointBi, finishedi, NewController[i][5], WorldTarget = NewController[i][1]. \
            planpath(list_i, obs_n[i], NewController[i][4], step, WorldTarget)

        acctEi, acclEi, NewController[i][4] = NewController[i][2]. \
            get_expected_action(obs_n[i], pointAi, pointBi, step, finishedi)

        actioni = NewController[i][3]. \
            get_action(obs_n[i], acctEi, acclEi, step, finishedi)

        action_n.append(actioni)

        if sight_friends[0]:
            # 存储 决策的个体编号 / 决策时状态 / 决策辅助信息 / 决策输出动作
            store_state.append([i, sight_friends[1], sight_friends[2], sight_friends[3]])

    return action_n, WorldTarget, store_state


def update_storage(arglist, store_state, new_obs_n, reward, trainer):

    def unitization(obs):

        vel = obs[0:2]
        length = np.sqrt(vel.dot(vel))
        v = vel / length

        return v

    if len(store_state) != len(reward):
        raise Exception('Cannot store training information. Please check store_state and reward!')

    for index, agent in enumerate(store_state):

        new_state_cache = np.zeros(arglist.agent_num*2)
        new_state_cache[0:2] = unitization(new_obs_n[agent[0]])

        for num in range(len(agent[2])):
            new_state_cache[(num+1)*2:(num+2)*2] = unitization(new_obs_n[agent[2][num]])

        trainer.store_transition(agent[1],
                                 np.array(agent[3]),
                                 reward[index],
                                 new_state_cache)

    store = True

    return store


def update_reward_1(arglist, area, last_cover, obs_n, episode_step):

    area_, last_cover_ = CR.update_area_cover(arglist.cover_edge, area, last_cover, obs_n, arglist.agent_num)
    cover_rate_, overlap_rate_ = CR.cal_cover_rate(area_)
    reward_ = (cover_rate_ - 1) * 100 / (episode_step + 1)

    return reward_, area_, last_cover_


def update_reward_2(arglist, area, last_cover, obs_n, median):

    area_, last_cover_ = CR.update_area_cover(arglist.cover_edge, area, last_cover, obs_n, arglist.agent_num)
    cover_rate_, overlap_rate_ = CR.cal_cover_rate(area_)
    reward_ = (cover_rate_ - median) * 1

    return reward_, area_, last_cover_


def update_reward_3(arglist, area, last_cover, obs_n, median):

    area_, last_cover_ = CR.update_area_cover(arglist.cover_edge, area, last_cover, obs_n, arglist.agent_num)
    cover_rate_, overlap_rate_ = CR.cal_cover_rate(area_)
    if cover_rate_ == median:
        reward_ = 0
    elif cover_rate_ > median:
        reward_ = 1
    else:
        reward_ = -1

    return reward_, area_, last_cover_


def update_reward_4(arglist, area, last_cover, obs_n):

    area_, last_cover_ = CR.update_area_cover(arglist.cover_edge, area, last_cover, obs_n, arglist.agent_num)
    cover_rate_, overlap_rate_ = CR.cal_cover_rate(area_)
    reward_ = cover_rate_

    return reward_, area_, last_cover_


def update_reward_5(arglist, area, last_cover, obs_n, episode_step, median):

    area_, last_cover_ = CR.update_area_cover(arglist.cover_edge, area, last_cover, obs_n, arglist.agent_num)
    cover_rate_, overlap_rate_ = CR.cal_cover_rate(area_)
    if cover_rate_ == median[episode_step // 5]:
        reward_ = 0
    elif cover_rate_ > median[episode_step // 5]:
        reward_ = 1
    else:
        reward_ = -1

    return reward_, area_, last_cover_


def update_reward_6(store_state):

    def cal_angle(vec):
        angle = math.atan2(vec[0], vec[1])
        angle_dgree = angle * 180 / math.pi
        if angle_dgree < 0:
            angle_dgree = 360 + angle_dgree
        return angle_dgree

    reward_set = list()

    for state_act in store_state:

        state = state_act[1]
        selfangle = cal_angle(state[0:2])
        length = state.size

        vel_angle = list()

        for index in range(2, length, 2):
            if state[index] == 0 and state[index + 1] == 0:
                break
            else:
                friend_angle = cal_angle(state[index:index + 2])
                reverse_angle = friend_angle + 180
                if reverse_angle >= 360:
                    reverse_angle -= 360
                vel_angle.append(friend_angle)
                vel_angle.append(reverse_angle)

        self_reverse = selfangle + 180
        if self_reverse >= 360:
            self_reverse -= 360
        vel_angle.append(self_reverse)

        act_angle = cal_angle(action_dict[str(int(state_act[3]))])

        distance_set = list()

        for ele in vel_angle:

            dis = abs(ele - act_angle)
            if dis > 180:
                dis = 360 - dis
            distance_set.append(dis)

        reward = min(distance_set) - 180

        reward_set.append(reward)

    return reward_set


