
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# --- Rotation matrices and axis rotation ---
def Rx(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[1,0,0],[0,c,-s],[0,s,c]])

def Ry(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]])

def rot_axis(u, theta):
    u = np.array(u, dtype=float); u /= np.linalg.norm(u)
    ux, uy, uz = u
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c + ux*ux*(1-c),    ux*uy*(1-c) - uz*s, ux*uz*(1-c) + uy*s],
        [uy*ux*(1-c) + uz*s, c + uy*uy*(1-c),    uy*uz*(1-c) - ux*s],
        [uz*ux*(1-c) - uy*s, uz*uy*(1-c) + ux*s, c + uz*uz*(1-c)]
    ])

def solve_theta_general(rC, rA, rAh, rBh, lrod, axis):
    v = np.asarray(rC) - np.asarray(rA)
    d = np.asarray(rBh) - np.asarray(rAh)
    RHS = (np.dot(v,v) + np.dot(d,d) - lrod**2)/2.0
    u = np.asarray(axis,dtype=float); u/=np.linalg.norm(u)
    d_par = u*np.dot(u,d)
    d_perp = d - d_par
    a = np.dot(v,d_perp)
    b = np.dot(v,np.cross(u,d))
    C = RHS - np.dot(v,d_par)
    D = a*a + b*b
    disc = b*b*C*C - D*(C*C - a*a)
    if disc < -1e-9: return []
    disc = max(disc,0.0)
    root = np.sqrt(disc)
    sin_candidates = [(b*C + root)/D, (b*C - root)/D]
    solutions=[]
    for s in sin_candidates:
        if abs(s) > 1.0+1e-8: continue
        s = float(np.clip(s,-1.0,1.0))
        cos_val = (C - b*s)/a if abs(a)>1e-12 else np.sqrt(max(0,1-s*s))
        theta = np.arctan2(s,cos_val)
        solutions.append(theta)
    return solutions

def angle_between(v1, v2):
    v1, v2 = np.array(v1), np.array(v2)
    cos_theta = np.dot(v1,v2)/(np.linalg.norm(v1)*np.linalg.norm(v2))
    return np.degrees(np.arccos(np.clip(cos_theta, -1.0, 1.0)))

def workspace(l_fixed=25, l_crank=75.0, l_rod=115.0,
                       l_spacing_rod=80.0, l_spacing_crank=80.0,
                       l_spacing_base=80.0, l_spacing_torso=80.0,
                       crank_angle_deg=0.0, rod_angle_deg=90.0,
                       pitch_deg=0.0, roll_deg=0.0):
 
    crank_angle = np.radians(crank_angle_deg)
    rod_angle   = np.radians(rod_angle_deg)
    pitch       = -np.radians(pitch_deg)
    roll        = -np.radians(roll_deg)

    # Home positions
    rA1_H = np.array([ l_spacing_crank/2.0, 0.0, l_rod - l_fixed + (l_crank * np.sin(crank_angle))])
    rA2_H = np.array([-l_spacing_crank/2.0, 0.0, l_rod - l_fixed + (l_crank * np.sin(crank_angle))])

    rB1_H = np.array([ l_spacing_rod/2.0, l_crank * np.cos(crank_angle), l_rod - l_fixed])
    rB2_H = np.array([ -l_spacing_rod/2.0, l_crank * np.cos(crank_angle), l_rod - l_fixed])

    rC1_H = np.array([ l_spacing_base/2.0, (l_crank * np.cos(crank_angle)) + (l_rod * np.cos(rod_angle)), - l_fixed])
    rC2_H = np.array([-l_spacing_base/2.0, (l_crank * np.cos(crank_angle)) + (l_rod * np.cos(rod_angle)), - l_fixed])

    rD1_H = np.array([ l_spacing_torso/2.0, 0.0, 0.0])
    rD2_H = np.array([-l_spacing_torso/2.0, 0.0, 0.0])

    rA1 = Rx(pitch) @ Ry(roll) @ rA1_H
    rA2 = Rx(pitch) @ Ry(roll) @ rA2_H
    rD1 = Rx(pitch) @ Ry(roll) @ rD1_H
    rD2 = Rx(pitch) @ Ry(roll) @ rD2_H

    axis = (rA1 - rA2)/np.linalg.norm(rA1 - rA2)

    theta1_candidates = solve_theta_general(rC1_H, rA1, rA1_H, rB1_H, l_rod, axis)
    theta2_candidates = solve_theta_general(rC2_H, rA2, rA2_H, rB2_H, l_rod, axis)
    if not theta1_candidates or not theta2_candidates:
        return None, None

    theta1 = theta1_candidates[0]
    theta2 = theta2_candidates[0]

    rB1 = rA1 + rot_axis(axis, theta1) @ (rB1_H - rA1_H)
    rB2 = rA2 + rot_axis(axis, theta2) @ (rB2_H - rA2_H)

    motor1 = angle_between(rD1_H - rA1_H, rB1_H - rA1_H) - angle_between(rD1 - rA1, rB1 - rA1)
    motor2 = angle_between(rD2_H - rA2_H, rB2_H - rA2_H) - angle_between(rD2 - rA2, rB2 - rA2)

    TR_pitch = np.abs(motor1 / pitch_deg) if pitch_deg != 0 else None
    TR_roll  = np.abs(motor2 / roll_deg) if roll_deg != 0 else None

    return TR_pitch, TR_roll

if __name__ == "__main__":
    
    pitch_vals = np.linspace(-90, 90, 100)
    roll_vals  = np.linspace(-90, 90, 100)
    
    rod_lengths = [90, 100, 110]
    crank_lengths = [70, 80, 90]
    link_spacings = [50, 50, 50]

    fig, axes = plt.subplots(1, len(rod_lengths), figsize=(15, 5), sharex=True, sharey=True)

    target_pitch_min, target_pitch_max = -10, 30
    target_roll_min, target_roll_max = -30, 30

    for idx, l_rod in enumerate(rod_lengths):
        feasible_x, feasible_y = [], []
        infeasible_x, infeasible_y = [], []

        for pitch in pitch_vals:
            for roll in roll_vals:
                TR_pitch, TR_roll = workspace(l_rod=l_rod, pitch_deg=pitch, roll_deg=roll)

                if TR_pitch is None and TR_roll is None:
                    # Non-feasible configuration
                    infeasible_x.append(pitch)
                    infeasible_y.append(roll)
                else:
                    # Feasible configuration
                    feasible_x.append(pitch)
                    feasible_y.append(roll)

        ax = axes[idx]
        ax.scatter(feasible_x, feasible_y, c='green', s=10)
        ax.scatter(infeasible_x, infeasible_y, c='red', s=10)

        # Add target workspace box
        rect = patches.Rectangle(
            (target_pitch_min, target_roll_min),
            target_pitch_max - target_pitch_min,
            target_roll_max - target_roll_min,
            linewidth=4,
            edgecolor='black',
            facecolor='none',
            linestyle='-'
        )
        ax.add_patch(rect)

        ax.set_title(f"l_rod = {l_rod}")
        ax.set_xlabel("Pitch (deg)")
        if idx == 0:
            ax.set_ylabel("Roll (deg)")
        ax.grid(True)

    plt.suptitle("Feasible vs Non-Feasible Workspace")
    plt.show()
