import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from scipy.optimize import differential_evolution

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
def mechanical_advantage(l_fixed = 15, l_crank = 70.0, l_rod = 90.0,
                        crank_angle_deg = 0.0,pitch_deg = 0.0, roll_deg = 0.0):
    
    l_spacing_rod=60.0
    l_spacing_crank=60.0
    l_spacing_base=60.0
    l_spacing_torso=60.0
    rod_angle_deg=90.0
    crank_angle = np.radians(crank_angle_deg)
    rod_angle   = np.radians(rod_angle_deg)
    pitch       = -np.radians(pitch_deg)
    roll        = -np.radians(roll_deg)

    # Home positions
    rA1_H = np.array([ l_spacing_crank/2.0, 0.0, l_rod - l_fixed + (l_crank * np.sin(crank_angle))])
    rA2_H = np.array([-l_spacing_crank/2.0, 0.0, l_rod - l_fixed + (l_crank * np.sin(crank_angle))])
    rB1_H = np.array([ l_spacing_rod/2.0, l_crank * np.cos(crank_angle), l_rod - l_fixed])
    rB2_H = np.array([-l_spacing_rod/2.0, l_crank * np.cos(crank_angle), l_rod - l_fixed])
    rC1_H = np.array([ l_spacing_base/2.0, (l_crank * np.cos(crank_angle)) + (l_rod * np.cos(rod_angle)), - l_fixed])
    rC2_H = np.array([-l_spacing_base/2.0, (l_crank * np.cos(crank_angle)) + (l_rod * np.cos(rod_angle)), - l_fixed])
    rD1_H = np.array([ l_spacing_torso/2.0, 0.0, 0.0])
    rD2_H = np.array([-l_spacing_torso/2.0, 0.0, 0.0])

    # Rotated torso points
    rA1 = Rx(pitch) @ Ry(roll) @ rA1_H
    rA2 = Rx(pitch) @ Ry(roll) @ rA2_H
    rD1 = Rx(pitch) @ Ry(roll) @ rD1_H
    rD2 = Rx(pitch) @ Ry(roll) @ rD2_H

    axis = (rA1 - rA2)/np.linalg.norm(rA1 - rA2)

    # Solve linkage
    theta1_candidates = solve_theta_general(rC1_H, rA1, rA1_H, rB1_H, l_rod, axis)
    theta2_candidates = solve_theta_general(rC2_H, rA2, rA2_H, rB2_H, l_rod, axis)
    if not theta1_candidates or not theta2_candidates:
        return None

    theta1 = theta1_candidates[0]
    theta2 = theta2_candidates[0]

    # Final B positions
    rB1 = rA1 + rot_axis(axis, theta1) @ (rB1_H - rA1_H)
    rB2 = rA2 + rot_axis(axis, theta2) @ (rB2_H - rA2_H)

    # Motor angles
    motor1 = angle_between(rD1_H - rA1_H, rB1_H - rA1_H) - angle_between(rD1 - rA1, rB1 - rA1)
    motor2 = angle_between(rD2_H - rA2_H, rB2_H - rA2_H) - angle_between(rD2 - rA2, rB2 - rA2)

    MA_pitch = np.abs(motor1 / pitch_deg) if pitch_deg != 0 else None
    return MA_pitch

def objective(x):
    l_fixed, l_crank, l_rod, crank_angle_deg = x
    MA = mechanical_advantage( l_fixed=l_fixed, l_crank=l_crank, l_rod=l_rod,
                              crank_angle_deg=crank_angle_deg, pitch_deg=10.0,roll_deg=0.0)
    if MA is None or np.isnan(MA):
        return 1e6  # infeasible - penalty
    return -MA  # negative for maximization

if __name__ == "__main__":
   
    bounds = [
        (0, 15), # l_fixed
        (70, 100), # l_crank
        (50, 100), # l_rod
        (0, 15) # crank_angle_deg
    ]

    x0 = [0, 70, 50, 0] # initial guess

    print("optimization to maximize mechanical advantage...")
    
    # # Sequential Least Squares Quadratic Programming
    # result = minimize(objective, x0, method='SLSQP', bounds=bounds,
    #                   options={'disp': True, 'ftol':1e-6})
    
    # Limited-memory Broyden–Fletcher–Goldfarb–Shanno with Bounds
    result = minimize(objective, x0, method='L-BFGS-B', bounds=bounds)
    
    # # Differential Evolution
    # result = differential_evolution( objective, bounds=bounds,strategy='best1bin',maxiter=1000,
    #                                 popsize=15, tol=1e-6, polish=True, disp=False )
    
    if result.success:
        best_l_fixed, best_l_crank, best_l_rod, best_crank_angle_deg = result.x
        max_MA = -result.fun
        print("Optimization successful!\n")
        print(f"Optimal link parameters:")
        print(f"  l_fixed          = {best_l_fixed:.3f}")
        print(f"  l_crank          = {best_l_crank:.3f}")
        print(f"  l_rod            = {best_l_rod:.3f}")
        print(f"  crank_angle_deg  = {best_crank_angle_deg:.3f}\n")
        print(f"Maximum Mechanical Advantage = {max_MA:.3f}")
    else:
        print("Optimization failed:", result.message)