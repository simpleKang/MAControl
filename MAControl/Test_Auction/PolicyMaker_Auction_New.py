from MAControl.Base.PolicyMaker import PolicyMaker
import math
import random
import numpy as np
from MAControl.Util.PointInRec import point_in_rec


class PolicyMaker_Auciton(PolicyMaker):

    # change TESTControl to PolicyMaker_Auciton
    Found_Target_Set = []
    Found_Target_Info = []
    Shared_UAV_state = []
    Shared_Big_Check = False
    Selectable_UAV = []
    Target_is_sorted = False  # Resorted_Target是否已进行排序
    Resorted_Target = []      # 按优先级排序的拍卖目标
    Auctioneer = -1           # 选出的拍卖者编号
    Target_index = -1         # 当前进行拍卖的目标编号

    Winner = []               # 最终选出来的优胜者列表
    Price_list = []           # 选出的竞拍者发出的竞拍价格
    unassigned_list = []      # 没有分到任务的个体列表

    last_step = 0
    Trans_step = []           # 拍卖者发送出目标给竞拍者的延时step列表
    wait_step = 30            # 等待的时长
    wait_step_auction = 10    # 选拍卖者的等待时间
    Update_step = 0

    def __init__(self, name, env, world, agent_index, arglist):
        super(PolicyMaker_Auciton, self).__init__(name, env, world, agent_index, arglist)
        self.detect_dis = 0.05
        self.comm_dis = 0.5
        self.close_area = []
        self.trans_step = []
        self.opt_index = 0
        self.x = 0
        self.y = 0

        PolicyMaker_Auciton.Shared_UAV_state.append(0)
        PolicyMaker_Auciton.unassigned_list.append(self.index)

    def find_mate(self, obs_n, R):
        selfpos = np.array(obs_n[self.index][2:4])
        close_area = []
        for i in range(len(obs_n)):
            posi = obs_n[i][2:4]
            deltapos = np.sqrt(np.dot(selfpos - posi, selfpos - posi))
            if deltapos < R:
                close_area.append(i)
        return close_area

    def add_new_target(self, obs, WorldTarget):
        TT_range = 0.05

        # COMPUTE selfview
        selfvel = np.array(obs[0:2])
        selfpos = np.array(obs[2:4])
        selfvelunit = selfvel / np.sqrt(np.dot(selfvel, selfvel))
        selfvelrightunit = np.array([selfvelunit[1], -1 * selfvelunit[0]])
        d1 = 0
        d2 = 0.5
        d3 = 0.5
        selfview1 = selfpos + selfvelunit * (d1 + d2) - selfvelrightunit * d3 / 2
        selfview2 = selfpos + selfvelunit * (d1 + d2) + selfvelrightunit * d3 / 2
        selfview3 = selfpos + selfvelunit * d1 + selfvelrightunit * d3 / 2
        selfview4 = selfpos + selfvelunit * d1 - selfvelrightunit * d3 / 2

        # GENERATE seen_target
        seen_target = []
        for target in WorldTarget:
            targetpos = np.array(target[1:3])
            if point_in_rec(selfview1, selfview2, selfview3, selfview4, targetpos):
                seen_target.append(target)

        # READ AND WRITE TESTControl.Found_Target_Set
        if PolicyMaker_Auciton.Found_Target_Set == []:
            PolicyMaker_Auciton.Found_Target_Set = seen_target
            for i in range(len(seen_target)):
                PolicyMaker_Auciton.Found_Target_Info.append(self.close_area)
        elif seen_target != []:
            for target1 in seen_target:
                check = False
                for target2 in PolicyMaker_Auciton.Found_Target_Set:
                    pos1 = np.array(target1[1:3])
                    pos2 = np.array(target2[1:3])
                    deltapos = np.sqrt(np.dot(pos1 - pos2, pos1 - pos2))
                    check = check | (deltapos <= TT_range)
                if not check:
                    PolicyMaker_Auciton.Found_Target_Set.append(target1)
                    PolicyMaker_Auciton.Found_Target_Info.append(self.close_area)

        # COMMUNICATE TESTControl.Found_Target_Info
        for info in PolicyMaker_Auciton.Found_Target_Info:
            check = False
            for num in self.close_area:
                check = check | num in info
            if check and (self.index not in info):
                info.append(self.index)

    def clearlist(self, step):
        PolicyMaker_Auciton.Selectable_UAV.clear()
        PolicyMaker_Auciton.Trans_step.clear()
        PolicyMaker_Auciton.Price_list.clear()
        PolicyMaker_Auciton.Auctioneer = -1
        PolicyMaker_Auciton.Resorted_Target.pop(0)
        PolicyMaker_Auciton.Update_step = step
        PolicyMaker_Auciton.wait_step = 30
        PolicyMaker_Auciton.wait_step_auction = 10

    def auction(self, obs, found_targets):
        # TODO 计算当前竞拍价格
        price = random.random()
        return price

    def make_policy(self, WorldTarget, obs_n, step):
        self.close_area = self.find_mate(obs_n, self.comm_dis) #更新邻域个体
        # 搜索阶段
        if PolicyMaker_Auciton.Shared_UAV_state[self.index] == 0: # =0 搜索
            if PolicyMaker_Auciton.Shared_Big_Check is True and PolicyMaker_Auciton.last_step == step - 1:
                PolicyMaker_Auciton.Shared_UAV_state[self.index] = 1 # 竞选拍卖者
                if PolicyMaker_Auciton.Target_is_sorted is False:
                    for i in range(len(PolicyMaker_Auciton.Found_Target_Set)):
                        PolicyMaker_Auciton.Resorted_Target.append([i, PolicyMaker_Auciton.Found_Target_Set[i][4], 0])
                    PolicyMaker_Auciton.Resorted_Target = sorted(PolicyMaker_Auciton.Resorted_Target, key=lambda x: x[1], reverse=True)
                    PolicyMaker_Auciton.Target_index = PolicyMaker_Auciton.Resorted_Target[0][0]
                    PolicyMaker_Auciton.Target_is_sorted = True
            else:
                # TODO 进行各种条件的计算判断，输出单个小飞机的大判断计算结果
                if step > 1500 and len(PolicyMaker_Auciton.Found_Target_Set) != 0:
                    PolicyMaker_Auciton.Shared_Big_Check = True
                    PolicyMaker_Auciton.last_step = step
                self.add_new_target(obs_n[self.index], WorldTarget)
        # 精选拍卖者
        elif PolicyMaker_Auciton.Shared_UAV_state[self.index] == 1:  # =1 选拍卖者
            if PolicyMaker_Auciton.Update_step == step - 1 and len(PolicyMaker_Auciton.Resorted_Target) != 0:
                PolicyMaker_Auciton.Target_index = PolicyMaker_Auciton.Resorted_Target[0][0]
                print('deal with next target')
            if len(PolicyMaker_Auciton.Resorted_Target) != 0:
                # 目标状态为0时进行拍卖者的选择，所有人都会进来
                if PolicyMaker_Auciton.Resorted_Target[0][2] == 0:
                    if PolicyMaker_Auciton.wait_step_auction > 0:
                        if self.index in PolicyMaker_Auciton.Found_Target_Info[PolicyMaker_Auciton.Target_index]:
                            # 进入到这里，说明k具备成为拍卖者的基本条件（知道这个目标，并且并不是正在攻击其他目标）
                            if True:  # TODO 这里可以增加个性化的条件，[满足基本条件但不满足这个条件的UAV]拒绝成为拍卖者，[条件都满足的UAV]向拍卖列表中添加自己的序号
                                PolicyMaker_Auciton.Selectable_UAV.append(self.index)
                        if self.index == PolicyMaker_Auciton.unassigned_list[-1]: # 最后一个未分配小飞机
                            if len(PolicyMaker_Auciton.Selectable_UAV) != 0:
                                # TODO 从列表中随机取个体作为拍卖者
                                PolicyMaker_Auciton.Auctioneer = random.choice(PolicyMaker_Auciton.Selectable_UAV)
                                PolicyMaker_Auciton.Resorted_Target[0][2] = 1
                            else:
                                PolicyMaker_Auciton.wait_step_auction -= 1
                    else:
                        PolicyMaker_Auciton.Selectable_UAV = PolicyMaker_Auciton.Found_Target_Info[PolicyMaker_Auciton.Target_index][:]
                        for i in PolicyMaker_Auciton.Selectable_UAV:
                            if PolicyMaker_Auciton.Shared_UAV_state[i] == 3: # 排除正在执行任务
                                PolicyMaker_Auciton.Selectable_UAV.remove(i)
                        if len(PolicyMaker_Auciton.Selectable_UAV) != 0:
                            # TODO 从列表中随机取个体作为拍卖者
                            PolicyMaker_Auciton.Auctioneer = random.choice(PolicyMaker_Auciton.Selectable_UAV)
                            PolicyMaker_Auciton.Resorted_Target[0][2] = 1
                        else:
                            print('没有小飞机能打这个目标了，放弃了')
                # 目标状态为1时所有人都会进来，看看自己是不是拍卖者，是的话进行操作，确认竞拍者及其延时step
                elif PolicyMaker_Auciton.Resorted_Target[0][2] == 1:
                    if self.index == PolicyMaker_Auciton.Auctioneer:
                        for i in self.close_area:
                            # TODO 传输延时step个数的计算优化
                            if PolicyMaker_Auciton.Shared_UAV_state[i] != 3: #不是正在执行任务
                                PolicyMaker_Auciton.Trans_step.append([i, round(math.sqrt(
                                    (obs_n[self.index][2] - obs_n[i][2]) ** 2 + (obs_n[self.index][3] - obs_n[i][3]) ** 2) / 0.05)])
                            PolicyMaker_Auciton.last_step = step
                            PolicyMaker_Auciton.Resorted_Target[0][2] = 2
                # 目标状态为2时更改竞拍者状态为2，在下一个step进行更新
                # TODO 优化
                elif PolicyMaker_Auciton.Resorted_Target[0][2] == 2 and (PolicyMaker_Auciton.last_step == step - 1):
                    for i in range(len(PolicyMaker_Auciton.Trans_step)):
                        if self.index == PolicyMaker_Auciton.Trans_step[i][0]:
                            PolicyMaker_Auciton.Shared_UAV_state[self.index] = 2
                            PolicyMaker_Auciton.Price_list.append([self.index])
                            self.trans_step = PolicyMaker_Auciton.Trans_step[i][1]
        # 竞价阶段
        elif PolicyMaker_Auciton.Shared_UAV_state[self.index] == 2:  #2 竞价
            if PolicyMaker_Auciton.Resorted_Target[0][2] == 2:
                # 拍卖者进入判断
                if self.index == PolicyMaker_Auciton.Auctioneer:
                    # 当拍卖者的等待时间完成时，根据价格选择优胜者
                    if PolicyMaker_Auciton.wait_step == 0:
                        if len(PolicyMaker_Auciton.Price_list) != 0:
                            PolicyMaker_Auciton.Price_list = sorted(PolicyMaker_Auciton.Price_list, key=lambda x: x[1], reverse=True)
                            if len(PolicyMaker_Auciton.Price_list) > PolicyMaker_Auciton.Found_Target_Set[PolicyMaker_Auciton.Target_index][5]:
                                for i in range(PolicyMaker_Auciton.Found_Target_Set[PolicyMaker_Auciton.Target_index][5]):
                                    PolicyMaker_Auciton.Winner.append(PolicyMaker_Auciton.Price_list[i][0])
                            else:
                                for i in range(len(PolicyMaker_Auciton.Price_list)):
                                    PolicyMaker_Auciton.Winner.append(PolicyMaker_Auciton.Price_list[i][0])
                            # 生成优胜者列表后目标状态置为3
                            PolicyMaker_Auciton.Resorted_Target[0][2] = 3
                            PolicyMaker_Auciton.last_step = step
                    else:
                        PolicyMaker_Auciton.wait_step -= 1
                # 所有竞拍者（拍卖者也是竞拍者）进入
                # TODO 已经可以重复添加价格了
                for i in range(len(PolicyMaker_Auciton.Price_list)):
                    if self.index == PolicyMaker_Auciton.Price_list[i][0]:
                        if self.trans_step == 0 and PolicyMaker_Auciton.wait_step > 0:
                            PolicyMaker_Auciton.Price_list[i].append(self.auction(obs_n[self.index], PolicyMaker_Auciton.Found_Target_Set))
                        else:
                            PolicyMaker_Auciton.Price_list[i].append(0)
                            self.trans_step -= 1
            elif PolicyMaker_Auciton.Resorted_Target[0][2] == 3:
                if PolicyMaker_Auciton.last_step == step - 1:
                    PolicyMaker_Auciton.Shared_UAV_state[self.index] = 1
                    if self.index in PolicyMaker_Auciton.Winner:
                        PolicyMaker_Auciton.Shared_UAV_state[self.index] = 3
                        # self.waypoint_list, self.current_wplist, self.pointB_index = W.attack_replace(
                        #     self.waypoint_list,
                        #     PolicyMaker_Auciton.Found_Target_Set[PolicyMaker_Auciton.Target_index][0:2], self.current_wplist)
                        self.opt_index = 5
                        self.x = PolicyMaker_Auciton.Found_Target_Set[PolicyMaker_Auciton.Target_index][0]
                        self.y = PolicyMaker_Auciton.Found_Target_Set[PolicyMaker_Auciton.Target_index][1]
                        PolicyMaker_Auciton.Winner.remove(self.index)
                        PolicyMaker_Auciton.unassigned_list.remove(self.index)
                        if len(PolicyMaker_Auciton.Winner) == 0:
                            self.clearlist(step)
                            if self.index < len(obs_n - 1):
                                for j in range(self.index, len(obs_n)):
                                    if PolicyMaker_Auciton.Shared_UAV_state[j] != 3:
                                        PolicyMaker_Auciton.Shared_UAV_state[j] = 1
        # 执行阶段
        elif PolicyMaker_Auciton.Shared_UAV_state[self.index] == 3:  #=3 执行
            self.opt_index = 0
            print('Agent_%d is attacking.' % self.index)

        return [self.opt_index, self.x, self.y]
