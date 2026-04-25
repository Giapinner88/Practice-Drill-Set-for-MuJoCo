import sympy as sp

# 1. Khai báo biến trạng thái
x, theta, dx, dtheta, u = sp.symbols('x theta dx dtheta u')
mc, mp, l, g = sp.symbols('mc mp l g')

# 2. Xây dựng phương trình Euler-Lagrange (M, C, G)
# Bản chất Inertial Coupling nằm ở thành phần mp*l*cos(theta)
M = sp.Matrix([
    [mc + mp, mp * l * sp.cos(theta)],
    [mp * l * sp.cos(theta), mp * l**2]
])

C = sp.Matrix([
    [-mp * l * dtheta**2 * sp.sin(theta)],
    [0]
])

G = sp.Matrix([
    [0],
    [-mp * g * l * sp.sin(theta)] # Gravity chỉ kéo con lắc
])

tau = sp.Matrix([[u], [0]]) # Lực u chỉ tác dụng vào xe (dof 1)

# Tính gia tốc: ddq = M^(-1) * (tau - C - G)
ddq = M.inv() * (tau - C - G)

# 3. Thành lập vector trường vector f(x, u)
# Trạng thái hệ thống: X = [x, theta, dx, dtheta]^T
f = sp.Matrix([
    dx,
    dtheta,
    ddq[0],
    ddq[1]
])

# 4. Tính Jacobian để tìm A và B (Tuyến tính hóa)
X = sp.Matrix([x, theta, dx, dtheta])
A_sym = f.jacobian(X)
B_sym = f.jacobian([u])

# 5. Thay số tại điểm cân bằng ngược (upright equilibrium)
# Trạng thái x* = 0, theta* = pi, vận tốc = 0, u* = 0
eq_dict = {x: 0, theta: sp.pi, dx: 0, dtheta: 0, u: 0, mc: 1.0, mp: 1.0, l: 0.5, g: 9.81}

A_num = A_sym.subs(eq_dict)
B_num = B_sym.subs(eq_dict)

print("Ma trận A (Analytical):")
sp.pprint(sp.N(A_num, 3))
print("\nMa trận B (Analytical):")
sp.pprint(sp.N(B_num, 3))