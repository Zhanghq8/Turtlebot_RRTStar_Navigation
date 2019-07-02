#!/usr/bin/env python

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

path = np.loadtxt('path.txt')
print (path[0])
smoothpath = np.loadtxt('smoothpath.txt')
print (smoothpath.shape)


x_list = path[0].tolist()
y_list = path[1].tolist()

x_list_smoothed = smoothpath[0].tolist()
y_list_smoothed = smoothpath[1].tolist()

plt.figure('Path figure')

ax = plt.gca()
ax.add_patch(
	patches.Rectangle(
		(-2.65,-2.2),   # (x,y)
		2.55,          # width
		0.7,          # height
		label='Obstacle'
	)	
)
ax.add_patch(
	patches.Rectangle(
		(0.75,-2.3),   # (x,y)
		1.7,          # width
		1.8,          # height
		label='Obstacle'
	)	
)
ax.add_patch(
	patches.Rectangle(
		(-3.9,0.75),   # (x,y)
		2.3,          # width
		0.8,          # height
		label='Obstacle'
	)	
)
ax.add_patch(
	patches.Rectangle(
		(1.85,1.65),   # (x,y)
		2.2,          # width
		0.85,          # height
		label='Obstacle'
	)	
)
ax.add_patch(
	patches.Rectangle(
		(-0.25,0.85),   # (x,y)
		0.7,          # width
		3.25,          # height
		label='Obstacle'
	)	
)
ax.set_xlim(left=-4, right=4)
ax.set_ylim(bottom=-4, top=4)

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_title('RRT Result')

ax.plot(x_list, y_list, color='r', marker='o', markersize=5, linewidth=2, alpha=0.6, label='Path without smooth')
ax.plot(x_list_smoothed, y_list_smoothed, color='g', marker='*', markersize=5, linewidth=2, alpha=0.6, label='Path after smooth')
ax.plot([path[0][0]], [path[1][0]], 'v', color='k', markersize=10, label='Start')
ax.plot([path[0][-1]], [path[1][-1]], '^', color='k', markersize=10, label='Goal')
ax.grid()

ax.legend()
plt.show()

