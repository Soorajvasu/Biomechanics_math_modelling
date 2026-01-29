from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np


def plot_plane(ax, p1, p2, p3, p4, color="cyan", alpha=0.5):
    verts = [[p1, p2, p3, p4]]
    plane = Poly3DCollection(verts, alpha=alpha, facecolor=color, edgecolor="k")
    ax.add_collection3d(plane)


def draw_axes(ax, origin=(0, 0, 0), L=0.1):
    ox, oy, oz = origin

    # X axis – Red
    ax.quiver(ox, oy, oz, L, 0, 0, color="r")
    ax.text(ox + L, oy, oz, "X", color="r")

    # Y axis – Green
    ax.quiver(ox, oy, oz, 0, L, 0, color="g")
    ax.text(ox, oy + L, oz, "Y", color="g")

    # Z axis – Blue
    ax.quiver(ox, oy, oz, 0, 0, L, color="b")
    ax.text(ox, oy, oz + L, "Z", color="b")
    
    
def set_equal_aspect(ax, points):
    all_pts = np.array(list(points.values()))
    x, y, z = all_pts[:,0], all_pts[:,1], all_pts[:,2]
    max_range = np.array([x.max()-x.min(),
                          y.max()-y.min(),
                          z.max()-z.min()]).max() / 2.0
    mid_x = (x.max() + x.min()) * 0.5
    mid_y = (y.max() + y.min()) * 0.5
    mid_z = (z.max() + z.min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
