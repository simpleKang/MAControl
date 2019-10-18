from MAControl.Base.PathPlanner import PathPlanner
import MAControl.Util.CreateWaypoint as CW
import math


class PathPlanner_generate_at_present(PathPlanner):

    def __init__(self, name, env, world, agent_index, arglist):
        super(PathPlanner_generate_at_present, self).__init__(name, env, world, agent_index, arglist)
        self.index = agent_index
        self.pointAi = (0, 0)         # A点坐标，即上一时刻已到达航点坐标
        self.pointBi = (0, 0)         # B点坐标，即此时待飞航点坐标
        # self.arrivals_current = 0     # 已经到达航点数
        self.arrivals_maxium = 50     # 最多到达多少次边界结束
        self.current_wplist = -1      # 当前航点列表的索引
        self.is_init = True           # 判断是否为初始时刻
        self.finished = False         # 是否到达最大循环数
        self.is_attacking = False     # 是否为正在执行
        self.waypoint_list = []       # 1×2的航点信息，每新增一个航点则增加一个列表，每个列表只存储一个航点

        self.waypoint_list, self.current_wplist = CW.creat_initial_waypoint_list\
            (world.agents[agent_index].state.p_pos, world.agents[agent_index].state.p_vel, 1)

    def planpath(self, para_list, obs, arrive_flag, step):
        if para_list[0] == 0:
            self.no_operation()

        elif para_list[0] == 10:
            # 攻击状态切换只能进来一次哦～～
            if self.is_attacking is False:
                self.attack_replace(para_list[1])
                self.pointAi = (obs[2], obs[3])
                self.pointBi = (self.waypoint_list[self.current_wplist][0],
                                self.waypoint_list[self.current_wplist][1])
                self.is_attacking = True
            else:
                pass
                # raise Exception('Target coord is changed again! This should not happen!!!')

        else:
            raise Exception('Unknown operation index. Please check your code.')

        # 初始时刻输出A、B坐标
        if self.is_init is True:
            self.pointAi = (obs[2], obs[3])
            self.pointBi = (self.waypoint_list[self.current_wplist][0],
                            self.waypoint_list[self.current_wplist][1])
            self.is_init = False

        if arrive_flag and self.is_attacking is False and self.finished is False:

            self.get_new_point()
            self.pointAi = (self.waypoint_list[self.current_wplist][0],
                            self.waypoint_list[self.current_wplist][1])
            self.pointBi = (self.waypoint_list[self.current_wplist + 1][0],
                            self.waypoint_list[self.current_wplist + 1][1])
            self.current_wplist += 1

            if self.current_wplist > self.arrivals_maxium:
                self.finished = True

        # 在攻击状态下到达航点时认为攻击目标成功，判断航点finish
        elif arrive_flag and self.is_attacking is True and self.finished is False:
            self.finished = True

        else:
            pass

        return self.pointAi, self.pointBi, self.finished, self.is_attacking

    # 操作数 = 0 不进行任何操作，返回当前航点列表
    def no_operation(self):
        pass

    # 操作数 = 10 攻击时刻的特殊操作，生成新的攻击列表，表中只有一行目标坐标
    def attack_replace(self, coord):
        self.waypoint_list.append([0 for i in range(2)])
        self.current_wplist += 1
        self.waypoint_list[self.current_wplist] = [coord[0], coord[1]]

    def get_new_point(self):
        self.waypoint_list.append([0 for i in range(2)])
        self.waypoint_list[-1] = CW.creat_random_edgepoint(self.waypoint_list[-2], 1)
