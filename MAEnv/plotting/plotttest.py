# rt plot test
from mpl_toolkits.mplot3d import axes3d

import matplotlib.pyplot as plt
import matplotlib.font_manager as font_manager
import numpy as np

# real-time (2d version )

POINTS = 100
sin_list = [0] * POINTS
indx = 0

fig, ax = plt.subplots()
ax.set_ylim([-2, 2])
ax.set_xlim([0, POINTS])
ax.set_autoscale_on(False)
ax.set_xticks(range(0, 100, 10))
ax.set_yticks(range(-2, 3, 1))
ax.grid(True)

line_sin, = ax.plot(range(POINTS), sin_list, label='Sin() output', color='cornflowerblue')
ax.legend(loc='upper center', ncol=4, prop=font_manager.FontProperties(size=10))


def sin_output(ax):
    global indx, sin_list, line_sin
    if indx == 20:
        indx = 0
    indx += 1

    sin_list = sin_list[1:] + [np.sin((indx / 10) * np.pi)]
    line_sin.set_ydata(sin_list)
    ax.draw_artist(line_sin)
    ax.figure.canvas.draw()


timer = fig.canvas.new_timer(interval=100)
timer.add_callback(sin_output, ax)
timer.start()
plt.show()


# real-time ( diy convert this to 3d ing ... )

POINTS = 100
sin_list = [0] * POINTS
cos_list = [0] * POINTS
indx = 0

fig = plt.figure(figsize=(12.3, 10))
ax = fig.add_subplot(1, 1, 1, projection='3d')
ax.set_xlim3d([0, POINTS])
ax.set_ylim3d([-2, 2])
ax.set_zlim3d([-2, 2])
ax.set_autoscale_on(False)
ax.set_xticks(range(0, 100, 10))
ax.set_yticks(range(-2, 3, 1))
ax.set_zticks(range(-2, 3, 1))
ax.grid(True)

line_sin_cos, = ax.plot(range(POINTS), sin_list, cos_list, label='Sin() cos() output', color='cornflowerblue')
ax.legend(loc='upper center', ncol=4, prop=font_manager.FontProperties(size=10))


def sin_cos_output(ax):
    global indx, sin_list, cos_list, line_sin_cos
    if indx == 20:
        indx = 0
    indx += 1

    sin_list = sin_list[1:] + [np.sin((indx / 10) * np.pi)]
    cos_list = cos_list[1:] + [np.cos((indx / 10) * np.pi)]
    line_sin_cos.set_ydata(sin_list)
    line_sin_cos.set_zdata(cos_list)
    ax.draw_artist(line_sin_cos)
    ax.figure.canvas.draw()


timer = fig.canvas.new_timer(interval=100)
timer.add_callback(sin_cos_output, ax)
timer.start()
plt.show()




