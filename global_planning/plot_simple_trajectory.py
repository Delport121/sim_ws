import trajectory_planning_helpers as tph
import numpy as np
import matplotlib.pyplot as plt
import os
import yaml


map_name = 'esp'
centreline = np.loadtxt(f"maps/{map_name}_centreline.csv", delimiter=',')

plt.figure()
plt.plot(centreline[:,0],centreline[:,1])
for i in range(len(centreline)):
    plt.arrow(
        centreline[i, 0],  # x-coordinate
        centreline[i, 1],  # y-coordinate
        0.5 * np.cos(centreline[i,4]),  # dx, based on heading angle
        0.5 * np.sin(centreline[i,4]),  # dy, based on heading angle 
        color='blue',
        head_width=0.05,  # Optional: adjust arrow head width
        length_includes_head=True,  # Optional: include head in length
    )
plt.show()


