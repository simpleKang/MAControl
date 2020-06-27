import numpy as np
import os
import matplotlib.pyplot as plt
import MAEnv.scenarios.TargetProfile as T
from PIL import Image
from matplotlib.patches import Circle, Wedge, Polygon
import math
from matplotlib.collections import PatchCollection

def coverrate_image_circle(gen, ind, num1):
    plt.rcParams['figure.dpi'] = 200

    curdir = os.path.dirname(__file__)
    pardir = os.path.dirname(os.path.dirname(curdir))

    para = np.loadtxt(pardir + '/track/para.txt')
    num = int(para[0])

    track = []
    for i in range(num):
        track.append(np.loadtxt(pardir + '/track/gen=%d/ind=%d/num=%d/uav_%d_track.txt' % (gen, ind, num1, i)))

    # for k in range(-10, 1501, 25):

    plt.figure(facecolor='w')
    line = plt.gca()
    line.set_aspect(1)
    line.patch.set_facecolor('white')
    plt.xlim(-(T.edge+0.2), T.edge+0.2)
    plt.ylim(-(T.edge+0.2), T.edge+0.2)
    edge = np.array(([T.edge, T.edge], [T.edge, -T.edge], [-T.edge, -T.edge], [-T.edge, T.edge], [T.edge, T.edge]))
    line.plot(edge[:, 0], edge[:, 1], 'r--')

    color = ['black', 'darkorange', 'forestgreen', 'slategrey', 'lightcoral', 'gold', 'mediumturquoise', 'darkviolet',
             'gray', 'burlywood', 'limegreen', 'cornflowerblue', 'firebrick', 'khaki', 'teal', 'plum',
             'silver', 'darkgoldenrod', 'lime', 'slateblue', 'red', 'yellow', 'cyan', 'purple',
             'lightgrey', 'gold', 'turquoise', 'blueviolet', 'darksalmon', 'darkseagreen', 'deepskyblue', 'hotpink']

    for i in range(num):
        k = i % len(color)
        #plt.scatter(track[i][0, 2], track[i][0, 3], c=color[k], marker='o')
        line.plot(track[i][:, 2], track[i][:, 3], color[k],marker='o',markersize=41.7)
        #line.plot(track[i][:, 2], track[i][:, 3], color[k])

    # if T.num_square:
    #     for s in range(T.num_square):
    #         plt.scatter(T.square_pos[s][0], T.square_pos[s][1], c='k', marker='*', linewidths=20)

    plt.xlabel('X / km')
    plt.ylabel('Y / km')
    plt.savefig(pardir + '/track/gen=%d/ind=%d/num=%d/track.png' %(gen,ind,num1))
    plt.close()

    plt.rcParams['figure.dpi'] = 200

    curdir = os.path.dirname(__file__)
    pardir = os.path.dirname(os.path.dirname(curdir))

    para = np.loadtxt(pardir + '/track/para.txt')
    num = int(para[0])

    track = []
    for i in range(num):
        track.append(np.loadtxt(pardir + '/track/gen=%d/ind=%d/num=%d/uav_%d_track.txt' % (gen, ind, num1, i)))

    # for k in range(-10, 1501, 25):

    plt.figure(facecolor='w')
    line = plt.gca()
    line.set_aspect(1)
    line.patch.set_facecolor('white')
    plt.xlim(-(T.edge + 0.2), T.edge + 0.2)
    plt.ylim(-(T.edge + 0.2), T.edge + 0.2)
    edge = np.array(([T.edge, T.edge], [T.edge, -T.edge], [-T.edge, -T.edge], [-T.edge, T.edge], [T.edge, T.edge]))
    line.plot(edge[:, 0], edge[:, 1], 'r--')

    color = ['black', 'darkorange', 'forestgreen', 'slategrey', 'lightcoral', 'gold', 'mediumturquoise', 'darkviolet',
             'gray', 'burlywood', 'limegreen', 'cornflowerblue', 'firebrick', 'khaki', 'teal', 'plum',
             'silver', 'darkgoldenrod', 'lime', 'slateblue', 'red', 'yellow', 'cyan', 'purple',
             'lightgrey', 'gold', 'turquoise', 'blueviolet', 'darksalmon', 'darkseagreen', 'deepskyblue', 'hotpink']

    for i in range(num):
        k = i % len(color)
        plt.scatter(track[i][0, 2], track[i][0, 3], c=color[k], marker='o')
        #line.plot(track[i][:, 2], track[i][:, 3], color[k], marker='o', markersize=41.7)
        line.plot(track[i][:, 2], track[i][:, 3], color[k])

    # if T.num_square:
    #     for s in range(T.num_square):
    #         plt.scatter(T.square_pos[s][0], T.square_pos[s][1], c='k', marker='*', linewidths=20)

    plt.xlabel('X / km')
    plt.ylabel('Y / km')
    plt.savefig(pardir + '/track/gen=%d/ind=%d/num=%d/track1.png' % (gen, ind, num1))
    plt.close()

    image1=Image.open(pardir + '/track/gen=%d/ind=%d/num=%d/track.png' %(gen,ind,num1))
    image=image1.crop([192,82,628,518])
    width=image.width
    height=image.height
    plt.imshow(image)
    plt.axis('off')
    plt.savefig('track-cut.png')
    plt.close()
    cover=0
    for i in range(0,width):
        for j in range(0,height):
            if image.getpixel((i,j))!=(255,255,255,255):
                cover += 1
    return cover/width/height

#计算扇形圆环的覆盖率
def coverrate_image_annular_sector():
    plt.rcParams['figure.dpi'] = 200

    curdir = os.path.dirname(__file__)
    pardir = os.path.dirname(os.path.dirname(curdir))

    para = np.loadtxt(pardir + '/track/para.txt')
    num = int(para[0])

    track = []
    for i in range(num):
        track.append(np.loadtxt(pardir + '/track/uav_%d_track.txt' % i))

    # for k in range(-10, 1501, 25):

    plt.figure(facecolor='w')
    line = plt.gca()
    line.set_aspect(1)
    line.patch.set_facecolor('white')
    plt.xlim(-(T.edge+0.2), T.edge+0.2)
    plt.ylim(-(T.edge+0.2), T.edge+0.2)
    edge = np.array(([T.edge, T.edge], [T.edge, -T.edge], [-T.edge, -T.edge], [-T.edge, T.edge], [T.edge, T.edge]))
    line.plot(edge[:, 0], edge[:, 1], 'r--')

    color = ['black', 'darkorange', 'forestgreen', 'slategrey', 'lightcoral', 'gold', 'mediumturquoise', 'darkviolet',
             'gray', 'burlywood', 'limegreen', 'cornflowerblue', 'firebrick', 'khaki', 'teal', 'plum',
             'silver', 'darkgoldenrod', 'lime', 'slateblue', 'red', 'yellow', 'cyan', 'purple',
             'lightgrey', 'gold', 'turquoise', 'blueviolet', 'darksalmon', 'darkseagreen', 'deepskyblue', 'hotpink']

    # if T.num_square:
    #     for s in range(T.num_square):
    #         plt.scatter(T.square_pos[s][0], T.square_pos[s][1], c='k', marker='*', linewidths=20)

    curdir = os.path.dirname(__file__)
    pardir = os.path.dirname(os.path.dirname(curdir))

    para = np.loadtxt(pardir + '/track/para.txt')
    num = int(para[0])

    # for k in range(-10, 1501, 25):
    cover = []
    ax = plt.subplot()
    for step in range(0, int(para[1]), 10):
        for num_uav in range(int(para[0])):
            angle = math.atan2(track[num_uav][step][1], track[num_uav][step][0]) / math.pi * 180
            cover += [
                Wedge((track[num_uav][step][2], track[num_uav][step][3]), 0.5, angle - 60, angle + 60, width=0.4,
                      color='yellow')]
        p = PatchCollection(cover, alpha=0.1)
        ax.add_collection(p)

    # if T.num_square:
    #     for s in range(T.num_square):
    #         plt.scatter(T.square_pos[s][0], T.square_pos[s][1], c='k', marker='*', linewidths=20)

    plt.xlabel('X / km')
    plt.ylabel('Y / km')
    plt.savefig(pardir + '/track/track.png' )
    plt.close()

    image1=Image.open(pardir + '/track/track.png' )
    image=image1.crop([192,82,628,518])
    width=image.width
    height=image.height
    plt.imshow(image)
    plt.axis('off')
    plt.savefig('track-cut.png')
    plt.close()
    cover=0
    for i in range(0,width):
        for j in range(0,height):
            if image.getpixel((i,j))!=(255,255,255,255):
                cover += 1
    return cover/width/height