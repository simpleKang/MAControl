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
        self.total_cycle = 3                # 列表循环的总次数
        self.current_wplist = -1            # 当前航点列表的索引
        self.path_pace = 50                 # PathPlanner的调用频率
        self.is_init = True                 # 是否为初始时刻
        self.is_attacking = False           # 是否为正在执行
        self.waypoint_finished = False      # 航点是否已经飞完
        # 初始化航点列表
        self.waypoint_list, self.current_wplist = CW.creat_snake_waypoint_list(
                                                  self.waypoint_list, self.env.n, self.index, self.current_wplist)

    def planpath(self, para_list, obs, arrive_flag, step):
        if para_list[0] == 0:
            self.no_operation()

        elif para_list[0] == 1:
            self.add(para_list[1])

        elif para_list[0] == 2:
            arrive_flag = self.insert(para_list[1])

        elif para_list[0] == 3:
            arrive_flag = self.replace(para_list[1])

        elif para_list[0] == 4:
            arrive_flag = self.delete(para_list[1])

        elif para_list[0] == 5:
            arrive_flag = self.complete_replace(para_list[1])

        elif para_list[0] == 10:
            # 攻击状态切换只能进来一次哦～～
            if self.is_attacking is False:
                self.attack_replace(para_list[1])
                self.pointAi = (obs[2], obs[3])
                self.pointBi = (self.waypoint_list[self.current_wplist][0][0],
                                self.waypoint_list[self.current_wplist][0][1])
                self.is_attacking = True
            else:
                pass
                # raise Exception('Target coord is changed again! This should not happen!!!')

        else:
            raise Exception('Unknown operation index. Please check your code.')

        # 初始时刻输出A、B坐标
        if self.pointB_index == 0 and self.is_init is True:
            self.pointAi = (obs[2], obs[3])
            self.pointBi = (self.waypoint_list[self.current_wplist][self.pointB_index][0],
                            self.waypoint_list[self.current_wplist][self.pointB_index][1])
            self.is_init = False

        # 到达B点后更新A、B点
        if arrive_flag and self.is_attacking is False and self.waypoint_finished is False:
            # 当下一个航点不为空且当前B点不为最后一个航点，直接取下一个航点作为B点
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
            # 当当前列表走完之后重新再走一遍列表
            elif self.cycle_index < self.total_cycle:
                for i in range(self.pointB_index + 1):
                    self.waypoint_list[self.current_wplist][i][2] = 1
                self.pointAi = (self.waypoint_list[self.current_wplist][self.pointB_index][0],
                                self.waypoint_list[self.current_wplist][self.pointB_index][1])
                self.pointBi = (self.waypoint_list[self.current_wplist][0][0],
                                self.waypoint_list[self.current_wplist][0][1])
                self.pointB_index = 0
                self.cycle_index += 1
            # 当当前列表循环次数达到设定值时，判断航点全部finish
            else:
                self.waypoint_finished = True

        # 在攻击状态下到达航点时认为攻击目标成功，判断航点finish
        elif arrive_flag and self.is_attacking is True and self.waypoint_finished is False:
            self.waypoint_finished = True

        else:
            pass
            
        return self.pointAi, self.pointBi, self.waypoint_finished, self.is_attacking

    # 操作数 = 0 不进行任何操作，返回当前航点列表
    def no_operation(self):
        pass

    # 操作数 = 1 参数形式 [1, add_list] add_list为N×2的坐标列表：添加航点操作
    def add(self, add_list):
        self.waypoint_list.append(self.waypoint_list[self.current_wplist].copy())
        self.current_wplist += 1

        # 当航点列表中存在空位时
        if [0, 0, 0] in self.waypoint_list[self.current_wplist]:
            end = self.waypoint_list[self.current_wplist].index([0, 0, 0])
            length = len(add_list)
            # 添加航点的个数小于航点列表中的空位，从第一个空位开始依次添加
            if length <= 256-end:
                for i in range(end, end+length):
                    self.waypoint_list[self.current_wplist][i] = [add_list[0][0], add_list[0][1], 1]
                    add_list.pop(0)
            # 添加航点的个数大于航点列表中的空位，把空位填满后再从航点列表第一行依次覆盖
            else:
                for i in range(end, 256):
                    self.waypoint_list[self.current_wplist][i] = [add_list[0][0], add_list[0][1], 1]
                    add_list.pop(0)
                for i in range(len(add_list)):
                    self.waypoint_list[self.current_wplist][i] = [add_list[i][0], add_list[i][1], 1]

        # 若航点列表已经满了，进行什么操作待定
        else:
            pass

    # 操作数 = 2 参数形式 [2, insert_list] insert_list为N×2的坐标列表：插入航点操作，在当前A、B点之间进行插入
    def insert(self, insert_list):
        self.waypoint_list.append(self.waypoint_list[self.current_wplist].copy())
        self.current_wplist += 1
        length = len(insert_list)

        # 当插入航点个数小于B点之后所有航点个数，从B点开始添加所有插入航点
        if length <= (256-self.pointB_index):
            for i in range(self.pointB_index, self.pointB_index+length):
                self.waypoint_list[self.current_wplist][i] = [insert_list[0][0], insert_list[0][1], 1]
                insert_list.pop(0)
            # 插入完成后，原B点及之后的航点再添加在插入航点的末尾，若达到256则停止，不再从起始位置覆盖
            end = self.pointB_index+length
            temp_index = self.pointB_index
            for i in range(end, 256):
                self.waypoint_list[self.current_wplist][i] = self.waypoint_list[self.current_wplist-1][temp_index]
                temp_index += 1

        # 当插入航点个数大于B点之后所有航点个数，从B点开始添加插入航点，并从起始位置覆盖继续添加插入航点，原B点及之后航点不再考虑
        else:
            for i in range(self.pointB_index, 256):
                self.waypoint_list[self.current_wplist][i] = [insert_list[0][0], insert_list[0][1], 1]
                insert_list.pop(0)
            for i in range(len(insert_list)):
                self.waypoint_list[self.current_wplist][i] = [insert_list[i][0], insert_list[i][1], 1]

        self.pointB_index -= 1

        return True

    # 操作数 = 3 参数形式 [3, replace_list] replace_list为N×2的坐标列表：替换航点操作，替换B点之后的航点
    def replace(self, replace_list):
        self.waypoint_list.append(self.waypoint_list[self.current_wplist].copy())
        self.current_wplist += 1

        length = len(replace_list)

        # 当替换列表的长度小于B点及之后航点的个数时，直接替换
        if length <= (256-self.pointB_index):
            for i in range(self.pointB_index, self.pointB_index+length):
                self.waypoint_list[self.current_wplist][i] = [replace_list[0][0], replace_list[0][1], 1]
                replace_list.pop(0)

        # 当替换列表的长度大于B点及之后航点的个数时，替换B点及之后航点个数的航点
        else:
            for i in range(self.pointB_index, 256):
                self.waypoint_list[self.current_wplist][i] = [replace_list[0][0], replace_list[0][1], 1]
                replace_list.pop(0)

        self.pointB_index -= 1

        return True

    # 操作数 = 4 参数形式 [4, num] num为从当前B点开始删除的航点个数：删除航点操作
    def delete(self, num):
        self.waypoint_list.append(self.waypoint_list[self.current_wplist].copy())
        self.current_wplist += 1

        # 当num小于B点及之后所有航点个数，删除B点及之后的num个航点
        if num <= (256-self.pointB_index):
            for i in range(num):
                self.waypoint_list[self.current_wplist].remove(self.waypoint_list[self.current_wplist][self.pointB_index])
                self.waypoint_list[self.current_wplist].append([0, 0, 0])

        # 当num大于B点及之后所有航点个数，删除B点及之后的所有航点
        else:
            for i in range(256-self.pointB_index):
                self.waypoint_list[self.current_wplist].remove(self.waypoint_list[self.current_wplist][self.pointB_index])
                self.waypoint_list[self.current_wplist].append([0, 0, 0])

        self.pointB_index -= 1

        return True

    # 操作数 = 5 参数形式 [5, [total_num, index]] 按total_num个个体生成第index个个体的蛇形航点列表
    def complete_replace(self, para_list):
        self.waypoint_list, self.current_wplist = CW.creat_snake_waypoint_list(
                                               self.waypoint_list, para_list[0], para_list[1], self.current_wplist)

        self.pointB_index = 0
        self.is_init = True

        return True

    # 操作数 = 10 攻击时刻的特殊操作，生成新的攻击列表，表中只有一行目标坐标
    def attack_replace(self, coord):
        self.waypoint_list.append([[0 for i in range(3)] for j in range(256)])
        self.current_wplist += 1
        self.waypoint_list[self.current_wplist][0] = [coord[0], coord[1], 1]
        self.pointB_index = 0

