from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import matplotlib.pyplot as plt


def draw_planes(ax, points, planes):
    for plane in planes:
        p1, p2, p3, p4 = plane["points"]

        verts = [
            [
                points[p1],
                points[p2],
                points[p3],
                points[p4],
            ]
        ]

        poly = Poly3DCollection(
            verts,
            alpha=plane.get("alpha", 0.3),
            facecolor=plane.get("color", "gray"),
            edgecolor="k",
        )

        ax.add_collection3d(poly)


def draw_segments(ax, points, segments, style, lw=2):
    for p1, p2 in segments:
        ax.plot(
            [points[p1][0], points[p2][0]],
            [points[p1][1], points[p2][1]],
            [points[p1][2], points[p2][2]],
            style,
            linewidth=lw,
        )


def draw_points(ax, points, point_style, size=30):
    for name, color in point_style.items():
        ax.scatter(*points[name], c=color, s=size)


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
    x, y, z = all_pts[:, 0], all_pts[:, 1], all_pts[:, 2]
    max_range = (
        np.array([x.max() - x.min(), y.max() - y.min(), z.max() - z.min()]).max() / 2.0
    )
    mid_x = (x.max() + x.min()) * 0.5
    mid_y = (y.max() + y.min()) * 0.5
    mid_z = (z.max() + z.min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)


def visualize_mechanism(
    points,
    point_style,
    links,
    fixed,
    home,
    planes,
):
    fig = plt.figure(figsize=(15, 5))

    def setup_axes(ax):
        ax.set_xlabel("X axis")
        ax.set_ylabel("Y axis")
        ax.set_zlabel("Z axis")
        ax.set_proj_type("ortho")
        set_equal_aspect(ax, points)
        draw_axes(ax, origin=(0, 0, 0), L=20)

    def plot_mechanism(ax):
        draw_points(ax, points, point_style)
        draw_segments(ax, points, links, "k-")
        draw_segments(ax, points, fixed, "b-")
        draw_segments(ax, points, home, "g--")
        setup_axes(ax)

    # ---- create views ----
    ax1 = fig.add_subplot(131, projection="3d")
    plot_mechanism(ax1)
    ax1.view_init(0, 90)

    ax2 = fig.add_subplot(132, projection="3d")
    plot_mechanism(ax2)
    ax2.view_init(20, 45)

    ax3 = fig.add_subplot(133, projection="3d")
    plot_mechanism(ax3)
    ax3.view_init(0, 0)

    # ---- planes per view ----
    ax_map = {"ax1": ax1, "ax2": ax2, "ax3": ax3}

    for plane in planes:
        for view in plane.get("views", []):
            draw_planes(ax_map[view], points, [plane])

    plt.tight_layout()
    plt.show()
