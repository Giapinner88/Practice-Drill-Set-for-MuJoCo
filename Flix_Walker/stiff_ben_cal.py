import numpy as np
from scipy.integrate import solve_bvp
import matplotlib.pyplot as plt  # Optional for plotting

# Full implementation of Algorithm 2 for straight HFM (uniform stiffness, straight beam)
# Based on paper: A Versatile Pseudo-Rigid Body Modeling Method
# Case: Straight catheter/guide-wire, parameters from Table I/II
# Updated load generation to match 3410 combinations as closely as possible: try f (0 to 4 mN, 11 points), psi (0 to 360 deg, 31 points), m_t (-0.25 to 0.25 N.m, 10 points) =11*31*10=3410

# Parameters
S = 0.05  # m (1000 mm)
#E = 350e6  # Pa (350 MPa)
#I = 5.7e-9  # m^4
EI = 0.00117  # N.m^2
n = 3  # DoF (change to 3,4,15,20 etc.)
f_max = 0.4  # N (4 N)
m_max = 0.025  # N.m (0.25 N.m)

# Generate grid loads to match 3410 combinations
f_points = np.linspace(-f_max, f_max, 10)  # 11 points: more zero-weighting, may reduce k_i
psi_points = np.linspace(0, 2 * np.pi, 10, endpoint=False)  # 31 points
mt_points = np.linspace(-m_max, m_max, 10)  # 31 points

w_list = []
for f in f_points:
    for psi in psi_points:
        for mt in mt_points:
            f_x = f * np.cos(psi)
            f_y = f * np.sin(psi)
            w_list.append([f_x, f_y, mt])

N = len(w_list)  # 3410
print(f"Generated {N} loading conditions")

l = S / n  # Equal segment lengths
s_points = np.linspace(0, S, n + 1)  # n+1 points

# ODE system for Eq. (4): straight uniform HFM
def ode(s, y, params):
    theta, thetap, x, ypos = y
    f, psi, m_t, EI_val = params
    dtheta = thetap
    dthetap = (f / EI_val) * np.sin(theta - psi)
    dx = np.cos(theta)
    dy = np.sin(theta)
    return np.vstack((dtheta, dthetap, dx, dy))

# Boundary conditions
def bc(ya, yb, params):
    f, psi, m_t, EI_val = params
    return np.array([ya[0], ya[2], ya[3], yb[1] - m_t / EI_val])

# Solve BVP for given load w = [f_x, f_y, m_t]
def solve_for_load(w, s_points, EI_val):
    f_x, f_y, m_t = w
    f = np.sqrt(f_x**2 + f_y**2)
    psi = np.arctan2(f_y, f_x) if f > 0 else 0.0
    params = (f, psi, m_t, EI_val)
    
    y_guess = np.zeros((4, len(s_points)))
    y_guess[2] = s_points
    y_guess[3] = np.zeros(len(s_points))
    
    sol = solve_bvp(lambda s, y: ode(s, y, params),
                    lambda ya, yb: bc(ya, yb, params),
                    s_points, y_guess, tol=1e-6, max_nodes=20000)  # Increased tol and nodes for precision
    
    if not sol.success:
        return None
    
    theta = sol.sol(s_points)[0]
    x = sol.sol(s_points)[2]
    y = sol.sol(s_points)[3]
    return theta, x, y

# Step 3: No-load (w=0)
w0 = [0.0, 0.0, 0.0]
sol0 = solve_for_load(w0, s_points, EI)
theta0_cont, x0, y0 = sol0 if sol0 is not None else (np.zeros(len(s_points)), s_points, np.zeros(len(s_points)))
theta0 = np.arctan2(np.diff(y0), np.diff(x0))

# Steps 6-9: Loaded cases (this may take time; consider parallelizing if needed)
Delta_phi_list = []
J_list = []
p_cont_list = []
w_success = []
for q, w in enumerate(w_list):
    sol_q = solve_for_load(w, s_points, EI)
    if sol_q is None:
        continue
    
    theta_q_cont, x_q, y_q = sol_q
    theta_q = np.arctan2(np.diff(y_q), np.diff(x_q))
    
    phi_q = np.diff(np.hstack(([0.0], theta_q)))
    
    Delta_phi_q = phi_q  # phi_0 = 0
    
    Delta_phi_list.append(Delta_phi_q)
    
    cum_theta = np.cumsum(phi_q)
    
    J = np.zeros((3, n))
    for m in range(n):
        sin_sum = np.sum(np.sin(cum_theta[m:]))
        cos_sum = np.sum(np.cos(cum_theta[m:]))
        J[0, m] = -l * sin_sum
        J[1, m] = l * cos_sum
        J[2, m] = 1.0
    
    J_list.append(J)
    
    p_cont_list.append([x_q[-1], y_q[-1], theta_q_cont[-1]])
    
    w_success.append(w)
    if (q + 1) % 500 == 0:
        print(f"Processed {q + 1}/{N} loads")

N_success = len(Delta_phi_list)
print(f"Successful loads: {N_success}/{N}")

Delta_phi = np.array(Delta_phi_list)

# Step 11: Optimal stiffness k_i
k = np.zeros(n)
for i in range(n):
    num = 0.0
    den = 0.0
    for q in range(N_success):
        J_i = J_list[q][:, i]
        dot = np.dot(J_i, w_success[q])
        num += dot * Delta_phi[q, i]
        den += Delta_phi[q, i]**2
    k[i] = num / den if den > 1e-20 else 0.0

print("Optimal stiffness k_i (N·m/rad):")
print(k)
