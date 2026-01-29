import numpy as np


def solve_theta(rC, rA, rB, lrod, axis):
    v = rC - rA
    d = rB - rA
    RHS = (np.dot(v, v) + np.dot(d, d) - lrod**2) / 2

    u = axis / np.linalg.norm(axis)
    d_par = u * np.dot(u, d)
    d_perp = d - d_par

    a = np.dot(v, d_perp)
    b = np.dot(v, np.cross(u, d))
    c = RHS - np.dot(v, d_par)

    D = a * a + b * b
    disc = b * b * c * c - D * (c * c - a * a)
    if disc < 0:
        return []

    root = np.sqrt(disc)
    sols = []

    for sth in [(b * c + root) / D, (b * c - root) / D]:
        if abs(sth) <= 1:
            cth = (c - b * sth) / a  # if abs(a)>1e-12 else np.sqrt(1-sth*sth)
            sols.append(np.arctan2(sth, cth))
    return sols
