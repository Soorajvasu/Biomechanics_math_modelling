import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def rot_axis(u, theta):
    """Rodrigues rotation matrix for rotation about arbitrary unit axis u."""
    u = np.array(u, dtype=float)
    u /= np.linalg.norm(u)
    ux, uy, uz = u
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c + ux*ux*(1-c),    ux*uy*(1-c) - uz*s, ux*uz*(1-c) + uy*s],
        [uy*ux*(1-c) + uz*s, c + uy*uy*(1-c),    uy*uz*(1-c) - ux*s],
        [uz*ux*(1-c) - uy*s, uz*uy*(1-c) + ux*s, c + uz*uz*(1-c)]
    ])
def Rx(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [1, 0, 0],
        [0, c, -s],
        [0, s,  c]
    ])
def Ry(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [ c, 0,  s],
        [ 0, 1,  0],
        [-s, 0,  c]
    ])
def Rz(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, -s, 0],
        [s,  c, 0],
        [0,  0, 1]
    ])

# Mechanism Parameters
lbar = 125.0     
lrod = 100.0     
lspacing = 50.0  

# Home position
O = np.array([0.0, 0.0, 0.0])
rA1 = np.array([ lspacing/2.0, 0.0, 175.0])
rA2 = np.array([-lspacing/2.0, 0.0, 175.0])
rB1_H = np.array([ lspacing/2.0, 100.0, 100.0])
rB2_H = np.array([-lspacing/2.0, 100.0, 100.0])
rC1_H = np.array([ lspacing/2.0, 100.0, 0.0])
rC2_H = np.array([-lspacing/2.0, 100.0, 0.0])
rD1 = np.array([ lspacing/2.0, 0.0, 0.0])
rD2 = np.array([-lspacing/2.0, 0.0, 0.0])

pitch = np.deg2rad(0.0)
roll  = np.deg2rad(0.0)

rC1 = Rx(pitch) @ Ry(roll) @ (rC1_H)
rC2 = Rx(pitch) @ Ry(roll) @ (rC2_H)

# Inverse Kinematics
axis = rA1 - rA2

def solve_theta_general(rC, rA, rB_home, lrod, axis):

    v = np.asarray(rC) - np.asarray(rA)
    d = np.asarray(rB_home) - np.asarray(rA)
    RHS = 0.5 * (np.dot(v, v) + np.dot(d, d) - lrod**2)

    u = np.asarray(axis, dtype=float)
    u /= np.linalg.norm(u)

    d_par = u * np.dot(u, d)
    d_perp = d - d_par
    a = np.dot(v, d_perp)
    b = np.dot(v, np.cross(u, d))
    C = RHS - np.dot(v, d_par)

    D = a*a + b*b
    disc = b*b*C*C - D*(C*C - a*a)
    if disc < -1e-9:
        return []                   # no real solutions (unreachable pose)
    disc = max(disc, 0.0)
    root = np.sqrt(disc)

    sin_candidates = [(b*C + root)/D, (b*C - root)/D]
    solutions = []
    for s in sin_candidates:
        if abs(s) > 1.0 + 1e-8:
            continue
        s = float(np.clip(s, -1.0, 1.0))
        cos_val = (C - b*s) / a if abs(a) > 1e-12 else np.sqrt(max(0, 1 - s*s))
        theta = np.arctan2(s, cos_val)
        solutions.append(theta)
    return solutions

theta1_candidates = solve_theta_general(rC1, rA1, rB1_H, lrod, axis)
theta2_candidates = solve_theta_general(rC2, rA2, rB2_H, lrod, axis)
theta1 = theta1_candidates[0]
theta2 = theta2_candidates[0]

rB1 = rA1 + rot_axis(axis, theta1) @ (rB1_H - rA1)
rB2 = rA2 + rot_axis(axis, theta2) @ (rB2_H - rA2)

final_position = {"O": O, "A1": rA1, "A2": rA2, "B1": rB1,
                  "C1": rC1, "B2": rB2, "C2": rC2, "D1": rD1, "D2": rD2, "B1H": rB1_H,
                 "B2H": rB2_H, "C1H": rC1_H, "C2H": rC2_H, "D1H": rD1, "D2H": rD2}

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
def plot_plane(ax, p1, p2, p3, p4, color='cyan', alpha=0.5):
    verts = [[p1, p2, p3, p4]]
    plane = Poly3DCollection(verts, alpha=alpha, facecolor=color, edgecolor='k')
    ax.add_collection3d(plane)
def visualize_mechanism(points):
    fig = plt.figure(figsize=(15, 5))
    fig.canvas.manager.set_window_title("Ankle RSU mech - parallel")

    def plot_mechanism(ax):
        for name, color in zip(["O","A1","A2","B1","B2","C1","C2","D1","D2","B1H","B2H",
                                "C1H","C2H","D1H","D2H"],
                               ['r','r','r','b','b','b','b','b','b','r','r','r','r','r','r']):
            ax.scatter(*points[name], c=color, label=name, s=30)

        links = [("A1","B1"),("A2","B2"),("B1","C1"),("B2","C2"),
                 ("C1","C2"),("C1","D1"),("C2","D2")]
        fixed = [("A1","A2"),("A2","D2"),("D2","D1"),("D1","A1")]
        Home = [("A1","B1H"),("A2","B2H"),("B1H","C1H"),("B2H","C2H"),
                 ("C1H","C2H"),("C1H","D1H"),("C2H","D2H")]
        for p1,p2 in links:
            ax.plot([points[p1][0], points[p2][0]],
                    [points[p1][1], points[p2][1]],
                    [points[p1][2], points[p2][2]], 'k-', linewidth=3)
        for p1,p2 in fixed:
            ax.plot([points[p1][0], points[p2][0]],
                    [points[p1][1], points[p2][1]],
                    [points[p1][2], points[p2][2]], 'b-', linewidth=3)
        for p1,p2 in Home:
            ax.plot([points[p1][0], points[p2][0]],
                    [points[p1][1], points[p2][1]],
                    [points[p1][2], points[p2][2]], 'g--', linewidth=2)

        ax.set_proj_type('ortho')
        set_equal_aspect(ax, points)

    # isometric, YZ view, XZ view
    ax1 = fig.add_subplot(132, projection='3d'); plot_mechanism(ax1); ax1.view_init(20, 45)
    ax2 = fig.add_subplot(131, projection='3d'); plot_mechanism(ax2); ax2.view_init(0, 90)
    ax3 = fig.add_subplot(133, projection='3d'); plot_mechanism(ax3); ax3.view_init(0, 0)

    plot_plane(ax1, points["A1"], points["A2"], points["D2"], points["D1"], color='cyan', alpha=0.5)
    plot_plane(ax1, points["D1"], points["C1"], points["C2"], points["D2"], color='yellow', alpha=0.5)
    plot_plane(ax2, points["A1"], points["A2"], points["D2"], points["D1"], color='cyan', alpha=0.5)
    plot_plane(ax2, points["D1"], points["C1"], points["C2"], points["D2"], color='yellow', alpha=0.5)

    plt.tight_layout()
def angle_between(v1, v2):
    v1, v2 = np.array(v1), np.array(v2)
    cos_theta = np.clip(np.dot(v1,v2) / (np.linalg.norm(v1)*np.linalg.norm(v2)),
                        -1.0, 1.0)
    return np.degrees(np.arccos(cos_theta))
def foot_velocity(rA1, rA2, rB1, rB2, rC1, rC2,
                                       theta_dot, axis1, axis2):

    rbar1 = rB1 - rA1
    rbar2 = rB2 - rA2
    rrod1 = rC1 - rB1
    rrod2 = rC2 - rB2

    s1 = np.array(axis1) / np.linalg.norm(axis1)
    s2 = np.array(axis2) / np.linalg.norm(axis2)

    J0 = np.diag([
        s1.dot(np.cross(rbar1, rrod1)),
        s2.dot(np.cross(rbar2, rrod2))
    ])

    Jx = np.vstack([
        np.hstack([rrod1, np.cross(rC1, rrod1)]),
        np.hstack([rrod2, np.cross(rC2, rrod2)])
    ])

    # For pure rotation about foot center (no translation)
    # v_O = 0, so we only keep the ω part (last 3 columns)
    Jx_omega = Jx[:, 3:]   # shape (2×3)

    # Solve Jx_omega * ω = Jθ * θ̇  for ω (least-squares)
    rhs = J0 @ np.array(theta_dot)
    omega, *_ = np.linalg.lstsq(Jx_omega, rhs, rcond=None)

    return omega
def compute_workspace(lbar, lrod, lspacing,
                      pitch_range=np.linspace(-60,60,61),
                      roll_range=np.linspace(-60,60,61)):
 
    O = np.array([0.0, 0.0, 0.0])
    rA1 = np.array([ lspacing/2.0, 0.0, 175.0])
    rA2 = np.array([-lspacing/2.0, 0.0, 175.0])
    rB1_H = np.array([ lspacing/2.0, 100.0, 100.0])
    rB2_H = np.array([-lspacing/2.0, 100.0, 100.0])
    rC1_H = np.array([ lspacing/2.0, 100.0, 0.0])
    rC2_H = np.array([-lspacing/2.0, 100.0, 0.0])
    axis = np.array([1.0,0.0,0.0])  # actuators rotate about X-axis in your frame

    feasible = []
    for pitch in pitch_range:
        for roll in roll_range:
            Rp = Rx(np.deg2rad(pitch))
            Rr = Ry(np.deg2rad(roll))
            R = Rp @ Rr

            rC1 = R @ (rC1_H - O)
            rC2 = R @ (rC2_H - O)

            sol1 = solve_theta_general(rC1, rA1, rB1_H, lrod, axis)
            sol2 = solve_theta_general(rC2, rA2, rB2_H, lrod, axis)

            if sol1 and sol2:  # both legs solvable
                feasible.append((roll, pitch))
    return feasible
def plot_workspace_variations():
    """Plot workspaces for different link parameter variations (like Fig.4)."""
    fig, ax = plt.subplots(figsize=(6,6))

    base = compute_workspace(lbar=125, lrod=100, lspacing=50)
    ax.scatter(*zip(*base), s=5, c='g', label='Baseline')

    long_bar = compute_workspace(lbar=125*2, lrod=100, lspacing=50)
    ax.scatter(*zip(*long_bar), s=5, c='b', label='Increase lbar')

    long_rod = compute_workspace(lbar=85, lrod=5*135, lspacing=43)
    ax.scatter(*zip(*long_rod), s=5, c='m', label='Increase lrod')

    wide_spacing = compute_workspace(lbar=85, lrod=135, lspacing=5*43)
    ax.scatter(*zip(*wide_spacing), s=5, c='y', label='Increase lspacing')

    ax.set_xlabel("Roll angle (deg)")
    ax.set_ylabel("Pitch angle (deg)")
    ax.set_title("Workspace vs Link Parameters")
    ax.legend()
    ax.grid(True)
    plt.show()

if __name__ == "__main__":
    
    print("θ1 solutions:", np.degrees(theta1_candidates))
    print("θ2 solutions:", np.degrees(theta2_candidates))
    print(f"Selected θ1: {np.degrees(theta1):.3f}°, θ2: {np.degrees(theta2):.3f}°")

    print("\nGeometry check:")
    print(f"Lowerlink: {np.linalg.norm(rC1 - rB1):.3f} ({lrod})")
    print(f"Upperlink: {np.linalg.norm(rA1 - rB1):.3f} ({lbar})")
    print(f"Spacing: {np.linalg.norm(rA1 - rA2):.3f} ({lspacing})")

    visualize_mechanism(final_position)
    plt.show()