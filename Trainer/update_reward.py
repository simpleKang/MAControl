import MAControl.Util.OnlineCoverRate as CR


def update_reward_1(arglist, area, last_cover, obs_n, episode_step):

    area_, last_cover_ = CR.update_area_cover(arglist.cover_edge, area, last_cover, obs_n, arglist.agent_num)
    cover_rate_, overlap_rate_ = CR.cal_cover_rate(area_)
    reward_ = (cover_rate_ - 1) * 100 / (episode_step + 1)

    return reward_, area_, last_cover_


def update_reward_2(arglist, area, last_cover, obs_n, median):

    area_, last_cover_ = CR.update_area_cover(arglist.cover_edge, area, last_cover, obs_n, arglist.agent_num)
    cover_rate_, overlap_rate_ = CR.cal_cover_rate(area_)
    reward_ = (cover_rate_ - median)*1

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
