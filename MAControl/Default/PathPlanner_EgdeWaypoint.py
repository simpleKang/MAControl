from MAControl.Base.PathPlanner import PathPlanner
import MAControl.Util.CreateWaypoint as CW
import numpy as np
import math


class PathPlanner_EgdeWaypoint(PathPlanner):

    AGENT_ALIVE = list()

    def __init__(self, name, env, world, agent_index, arglist):
        super(PathPlanner_EgdeWaypoint, self).__init__(name, env, world, agent_index, arglist)
        self.pointAi = (0, 0)         # A点坐标，即上一时刻已到达航点坐标
        self.pointBi = (0, 0)         # B点坐标，即此时待飞航点坐标
        self.edge = world.edge        # 区域边界，为一个象限的边长，即区域总边长为2×edge，单位km
        self.arrivals_maxium = 1000   # 最多到达多少次边界结束
        self.current_wplist = 0       # 当前航点列表的索引
        self.is_init = True           # 判断是否为初始时刻
        self.finished = False         # 是否到达最大循环数
        self.is_attacking = False     # 是否为正在执行
        self.waypoint_list = list()   # 1×2的航点信息，每新增一个航点则增加一个列表，每个列表只存储一个航点
        self.attacking_target = list()
        self.waypoint_list.append(CW.creat_veledge_point(world.agents[self.index].state.p_pos,
                                                         world.agents[self.index].state.p_vel, world.edge))

        PathPlanner_EgdeWaypoint.AGENT_ALIVE.append(True)

    def planpath(self, para_list, obs, arrive_flag, step, worldtarget):
        if para_list[0] == 0:
            self.no_operation()

        elif para_list[0] == 1:
            self.new_decision_point(para_list[1], obs[2:4])
            self.pointAi = (obs[2], obs[3])
            self.pointBi = (self.waypoint_list[self.current_wplist][0],
                            self.waypoint_list[self.current_wplist][1])

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

            # self.get_new_random_point()
            self.get_new_reflection_point(obs[0:2])
            self.pointAi = (self.waypoint_list[self.current_wplist][0],
                            self.waypoint_list[self.current_wplist][1])
            self.pointBi = (self.waypoint_list[self.current_wplist + 1][0],
                            self.waypoint_list[self.current_wplist + 1][1])
            self.current_wplist += 1

            if self.current_wplist > self.arrivals_maxium:
                self.finished = True
                PathPlanner_EgdeWaypoint.AGENT_ALIVE[self.index] = False

        # 在攻击状态下到达航点时认为攻击目标成功，判断航点finish
        elif arrive_flag and self.is_attacking is True and self.finished is False:
            self.finished = True
            target_index = worldtarget.index(self.attacking_target)
            worldtarget[target_index][3] -= 1
            PathPlanner_EgdeWaypoint.AGENT_ALIVE[self.index] = False

        else:
            pass

        return self.pointAi, self.pointBi, self.finished, self.is_attacking, worldtarget

    # 操作数 = 0 不进行任何操作，返回当前航点列表
    def no_operation(self):
        pass

    # 操作数 = 1 根据速度方向进行延长，取与边界交点
    def new_decision_point(self, v, pos):
        self.waypoint_list.append(CW.creat_veledge_point(pos, v, self.edge))
        self.current_wplist += 1

    # 操作数 = 10 攻击时刻的特殊操作，生成新的攻击列表，表中只有一行目标坐标
    def attack_replace(self, coord):
        self.waypoint_list.append([0 for i in range(2)])
        self.current_wplist += 1
        self.waypoint_list[-1] = [coord[0], coord[1]]

    # 随机生成新的航点
    def get_new_random_point(self):
        self.waypoint_list.append([0 for i in range(2)])
        self.waypoint_list[-1] = CW.creat_random_edgepoint(self.waypoint_list[-2], self.edge)

    # 生成反射航点
    def get_new_reflection_point(self, vel):
        self.waypoint_list.append([0 for i in range(2)])
        self.waypoint_list[-1] = CW.creat_reflection_edgepoint(self.waypoint_list[-2], vel, self.edge)
