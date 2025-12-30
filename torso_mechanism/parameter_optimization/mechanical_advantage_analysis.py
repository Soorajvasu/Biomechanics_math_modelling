
import numpy as np
import matplotlib.pyplot as plt

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

def mechanical_advantage(l_fixed=15, l_crank=70.0, l_rod=90.0,
                       l_spacing_rod=60.0, l_spacing_crank=60.0,
                       l_spacing_base=60.0, l_spacing_torso=60.0,
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
    
    # rod_angle = angle_between(rC1_H - rB1_H, np.array([0.0, 0.0, -15.0]) - rC1_H) - angle_between(rC1_H - rB1, np.array([0.0, 0.0, -15.0]) - rC1_H)

    MA_pitch = np.abs(motor1 / pitch_deg) if pitch_deg != 0 else None

    return MA_pitch, motor1 #, MA_roll

if __name__ == "__main__":

    values1 = np.linspace(-10, 25, 200)
    values2 = np.linspace(50, 120, 7)

    plt.figure(figsize=(8, 6))
    for l_crank in values2:
        MAs = []
        for pitch in values1:
            MA, _ = mechanical_advantage(l_crank=l_crank, pitch_deg=pitch)
            MAs.append(MA if MA is not None else np.nan)
        plt.plot(values1, MAs, label=f"l_crank = {l_crank:.0f} mm")

    plt.xlabel("Pitch angle (deg)")
    plt.ylabel("Mechanical Advantage")
    plt.title("Mechanical Advantage vs Pitch Angle")
    plt.legend()
    plt.grid(True)

    pitch_values = np.linspace(-45, 45, 180)
    
    motor_angles = []

    for pitch in pitch_values:
        _, motor_angle = mechanical_advantage(pitch_deg=pitch)
        motor_angles.append(motor_angle)

    plt.figure(figsize=(8, 6))
    plt.plot(pitch_values, motor_angles, color='r', label="Motor angle response")

    plt.xlabel("Pitch angle (deg)")
    plt.ylabel("Motor angle (deg)")
    plt.title("Motor Angle vs Pitch Angle")
    plt.legend()
    plt.grid(True)

    plt.show()