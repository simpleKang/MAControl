from MAControl.Base.PathPlanner import PathPlanner
import MAControl.Util.CreateWaypoint as CW


class PathPlanner_Simple(PathPlanner):

    def __init__(self, name, env, world, agent_index, arglist):
        super(PathPlanner_Simple, self).__init__(name, env, world, agent_index, arglist)
        self.waypoint_list = []             # 256×3的航点列表，第3列为航点状态 [0: 无航点] [1: 未飞] [2: pointA] [3: pointB] [4: 已到达]
        self.pointAi = (0, 0)               # A点坐标
        self.pointBi = (0, 0)               # B点坐标
        self.pointB_index = 0               # 当前飞向的B点的索引
        self.cycle_index = 1                # 航点列表循环的次数
        self.total_cycle = 2                # 列表循环的总次数
        self.current_wplist = 0             # 当前航点列表的索引
        self.path_pace = 50                 # PathPlanner的调用频率
        self.is_init = True                 # 是否为初始时刻
        self.is_attacking = False           # 是否为正在执行
        self.waypoint_finished = False      # 航点是否已经飞完

        self.waypoint_list.append([[0 for i in range(3)] for j in range(256)])
        self.waypoint_list[self.current_wplist][0:len(CW.init_waypoint[self.index])] = CW.init_waypoint[self.index]

    def planpath(self, para_list, obs, arrive_flag):
        if para_list[0] == 0:
            self.waypoint_list = self.no_operation(self.waypoint_list)

        elif para_list[0] == 1:
            pass

        elif para_list[0] == 2:
            pass

        elif para_list[0] == 3:
            pass

        elif para_list[0] == 4:
            pass

        elif para_list[0] == 5:
            self.waypoint_list, self.current_wplist, self.pointB_index = \
                self.attack_replace(self.waypoint_list, [para_list[1], para_list[2]], self.current_wplist)
            if self.is_attacking is False:
                self.pointAi = (obs[2], obs[3])
                self.pointBi = (self.waypoint_list[self.current_wplist][0][0],
                                self.waypoint_list[self.current_wplist][0][1])
                self.is_attacking = True
            else:
                raise Exception('Target coord is changed again! This should not happen!!!')

        # 初始时刻输出A、B坐标
        if self.pointB_index == 0 and self.is_init is True:
            self.pointAi = (obs[2], obs[3])
            self.pointBi = (self.waypoint_list[self.current_wplist][self.pointB_index][0],
                            self.waypoint_list[self.current_wplist][self.pointB_index][1])
            self.is_init = False

        # 更改航点状态并输出A、B坐标
        if arrive_flag and self.is_attacking is False and self.waypoint_finished is False:
            if self.waypoint_list[self.current_wplist][self.pointB_index+1][2] != 0 and self.pointB_index < 255:
                if self.pointB_index > 0:
                    self.waypoint_list[self.current_wplist][self.pointB_index-1][2] = 4
                self.waypoint_list[self.current_wplist][self.pointB_index][2] = 2
                self.waypoint_list[self.current_wplist][self.pointB_index+1][2] = 3
                self.pointAi = (self.waypoint_list[self.current_wplist][self.pointB_index][0],
                                self.waypoint_list[self.current_wplist][self.pointB_index][1])
                self.pointBi = (self.waypoint_list[self.current_wplist][self.pointB_index+1][0],
                                self.waypoint_list[self.current_wplist][self.pointB_index+1][1])
                self.pointB_index += 1

            else:
                if self.cycle_index < self.total_cycle:
                    for i in range(self.pointB_index + 1):
                        self.waypoint_list[self.current_wplist][i][2] = 1
                    self.pointAi = (self.waypoint_list[self.current_wplist][self.pointB_index][0],
                                    self.waypoint_list[self.current_wplist][self.pointB_index][1])
                    self.pointBi = (self.waypoint_list[self.current_wplist][0][0],
                                    self.waypoint_list[self.current_wplist][0][1])
                    self.pointB_index = 0
                    self.cycle_index += 1
                else:
                    self.waypoint_finished = True

        elif arrive_flag and self.is_attacking is True and self.waypoint_finished is False:
            self.waypoint_finished = True

        # else:
            # print('hai mei dao')
            
        return self.pointAi, self.pointBi, self.waypoint_finished

    # 操作数 = 0 不进行修改
    def no_operation(self, original):
        return original

    # TODO 操作数 = 1 添加操作
    def add(self, original, addlist):
        new = []
        return new

    # TODO 操作数 = 2 插入操作
    def insert(self, original, insertlist, pos):
        new = []
        return new

    # TODO 操作数 = 3 替换操作
    def replace(self, original):
        new = []
        return new

    # TODO 操作数 = 4 删除操作
    def delete(self, original, deleteindex):
        new = []
        return new

    # TODO 操作数 = 5 攻击时替换列表
    def attack_replace(self, original, coord, list_index):
        original.append([[0 for i in range(3)] for j in range(256)])
        list_index += 1
        original[list_index][0][0:3] = [coord[0], coord[1], 5]
        pointB_index = 0
        return original, list_index, pointB_index


