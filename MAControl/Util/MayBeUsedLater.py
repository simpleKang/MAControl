
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


