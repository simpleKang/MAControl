from MAControl.Base.PathPlanner import PathPlanner


class PathPlanner_Simple(PathPlanner):

    def __init__(self, name, env, world, agent_index, arglist):
        super(PathPlanner_Simple, self).__init__(name, env, world, agent_index, arglist)
        self.waypoint_list = []             # 256×3的航点列表，第3列为航点状态 [0: 无航点] [1: 未飞] [2: pointA] [3: pointB] [4: 已到达]
        self.pointAi = (0, 0)               # A点坐标
        self.pointBi = (0, 0)               # B点坐标
        self.pointB_index = 0               # 当前飞向的B点的索引
        self.cycle_index = 1                # 航点列表循环的次数
        self.total_cycle = 1                # 列表循环的总次数
        self.current_wplist = 0             # 当前航点列表的索引
        self.path_pace = 50                 # PathPlanner的调用频率
        self.is_init = True                 # 是否为初始时刻
        self.is_attacking = False           # 是否为正在执行
        self.arrive_flag = False            # 是否到达B点
        self.waypoint_finished = False      # 航点是否已经飞完

        self.waypoint_list.append([[0 for i in range(3)] for j in range(256)])

    def planpath(self):
        print('This is a pathplanner.')
        pass




    # TODO 不进行修改
    def no_operation(self, original):
        new = []
        return new

    # TODO 添加操作
    def add(self, original, addlist):
        new = []
        return new

    # TODO 插入操作
    def insert(self, original, insertlist, pos):
        new = []
        return new

    # TODO 替换操作
    def replace(self, original):
        new = []
        return new

    # TODO 删除操作
    def delete(self, original, deleteindex):
        new = []
        return new

    # TODO 攻击时替换列表
    def attack_replace(self, original, coord, list_index):
        original.append([[0 for i in range(3)] for j in range(256)])
        list_index += 1
        original[list_index][0][0:3] = [coord[0], coord[1], 5]
        pointB_index = 0
        return original, list_index, pointB_index


