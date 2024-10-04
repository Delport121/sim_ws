import trajectory_planning_helpers as tph
import numpy as np
import matplotlib.pyplot as plt
import os
import yaml
from scipy import interpolate

#To run this script be sure to be in the dircetory src/global_planning

def interpolate_track_new(points, n_points=None, s=0):
    if len(points) <= 1:
        return points
    order_k = min(3, len(points) - 1)
    tck = interpolate.splprep([points[:, 0], points[:, 1]], k=order_k, s=s)[0]
    if n_points is None: n_points = len(points)
    track = np.array(interpolate.splev(np.linspace(0, 1, n_points), tck)).T
    return track

def resample_track_points(points, seperation_distance=0.2, smoothing=0.2):
    if points[0, 0] > points[-1, 0]:
        # points = np.flip(points, axis=0)
        points = np.flip(points, axis=1)

    line_length = np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1))
    n_pts = max(int(line_length / seperation_distance), 2)
    smooth_line = interpolate_track_new(points, None, smoothing)
    resampled_points = interpolate_track_new(smooth_line, n_pts, 0)

    return resampled_points, smooth_line


map_name = 'gbr'
centreline = np.loadtxt(f"maps/{map_name}_centreline.csv", delimiter=',')
resampled_points, smooth_line = resample_track_points(centreline[:,:2], seperation_distance=0.1, smoothing=0.5)
print(f'Original Points: {len(centreline[:,:2])}')
print(f'Smothed Points: {len(smooth_line)}')
print(f'Resampled Points: {len(resampled_points)}')

el_lengthsSmooth = np.sqrt(np.sum(np.diff(smooth_line, axis=0)**2, axis=1))
psiSmooth, kappaSmooth = tph.calc_head_curv_num.calc_head_curv_num(
        path=smooth_line,
        el_lengths=el_lengthsSmooth,
        is_closed=False,
    )

el_lengthsResample = np.sqrt(np.sum(np.diff(resampled_points, axis=0)**2, axis=1))
psiResample, kappaResample = tph.calc_head_curv_num.calc_head_curv_num(
        path=resampled_points,
        el_lengths=el_lengthsResample,
        is_closed=False,
    )

plt.figure(0)
plt.title("Heading")
plt.plot(centreline[:,4], label="Raw" )
plt.plot(-psiSmooth, label="Smooth" )    #Note the minus here that flips it
plt.legend()
plt.show()

plt.figure(1)
plt.title("Curvature")
plt.plot(centreline[:,5], label="Raw" )
plt.plot(kappaSmooth, label="Smooth")
plt.legend()
plt.show()