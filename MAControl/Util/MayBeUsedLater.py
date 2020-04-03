
'''
这里临时存放一些在正式代码中暂时不使用,注释掉又显得不好看的鸡肋代码,万一以后要用就不用重新写了
'''

'''
execute.py
'''
# while True的外面
# if arglist.OnlineCoverRate:
#     last_cover = list()
#     area = np.zeros((arglist.cover_edge, arglist.cover_edge))
#     open('cover_rate.txt', 'w')
#     for k in range(env.n - len(world.movable_targets)):
#         last_cover.append([])

# while True的里面
# if arglist.OnlineCoverRate:
#     if step % 5 == 0:
#         area, last_cover = CR.update_area_cover(arglist.cover_edge, area, last_cover, obs_n,
#                                                 env.n - len(world.movable_targets))
#         cover_rate, overlap_rate = CR.cal_cover_rate(area)
#         with open('cover_rate.txt', 'a') as c:
#             c.write(str(step)+' '+str(cover_rate)+' '+str(overlap_rate)+'\n')
#         print('>>>> cover rate ', cover_rate)


# with open(os.path.dirname(__file__) + '/save_model/reward.txt', 'a') as f:
#   f.write(str(episode_step)+' '+str(reward)+'\n')

# 2020.03.13 KSB
# with open(os.path.dirname(__file__) + _path + 'gen=%d/ind=%d/num=%d/information.txt' % (gen, ind, num), 'a') as f:
                #     f.write(str(individual) +'    '+str(score))

# # 为每个小瓜子创建状态文件
    # os.makedirs(os.path.dirname(__file__) + _path + 'gen=%d/ind=%d/num=%d' % (gen, ind, num))
    # for k in range(arglist.uav_num):
    #     open(os.path.dirname(__file__) + _path + 'gen=%d/ind=%d/num=%d/uav_%d_track.txt'%(gen,ind,num,k), 'w')

# with open(os.path.dirname(__file__)+_path+'gen=%d/ind=%d/num=%d/uav_%d_track.txt'%(gen,ind,num,k),'a') as f:
            #     f.write(str(obs_n[k][0])+' '+str(obs_n[k][1])+' '+str(obs_n[k][2])+' '+str(obs_n[k][3])+'\n')


# PathPlanner_EdgeWaypoint
# with open(os.path.dirname(os.path.dirname(os.path.dirname(__file__))) + _path + 'waypoint_%d.txt' % self.index, 'a') as f:
#     f.write(str(1) + ' ' + str(self.waypoint_list[self.current_wplist][0]) + ' ' +
#                          str(self.waypoint_list[self.current_wplist][1]) + '\n')