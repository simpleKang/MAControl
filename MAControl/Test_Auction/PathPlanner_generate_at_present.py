from MAControl.Base.PathPlanner import PathPlanner
import math

class PathPLanner_generate_at_present(PathPlanner)

    def __init__(self,name,env,world,agent_index,arglist):
      super().__init__(name,env,world,agent_index,arglist)
      self.pointAi = (0,0)   #A点坐标，即上一时刻已到达航点坐标
      self.pointBi = (0,0)   #B点坐标，即此时待飞航点坐标
      self.is_init = True    #判断是否为初始时刻
      self.arrivals_current = 0   #已经到达航点数
      self.arrivals_maxium = 20   #最多到达多少次边界结束
      self.finished = False       #是否到达最大循环数
      self.current_wplist = []    #记录所有经历航点的列表，maxium*3的列表

    def generate_new_point(self,para_list,obs,arrive_flag,step):
        if para_list[0] == 0:       #等待下一步
           self.no_operation()

        elif para_list[0] == 1:     #设置下一循环
            self.add(para_list[1])

        elif para_list[0] == 2:
            arrive_flag = self.insert(para_list[1])    #立即中止搜索，下一步返回

        elif para_list[0] == 3:
            arrive_flag = self.replace(para_list[1])   #

        elif para_list[0] == 4:
            arrive_flag = self.delete(para_list[1])

        elif para_list[0] == 5:
            arrive_flag = self.complete_replace(para_list[1])