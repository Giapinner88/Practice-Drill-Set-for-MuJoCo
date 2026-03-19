import numpy as np
from scipy.integrate import solve_bvp

def calculate_flix_walker_stiffness_diversified_loads():
    # --- 1. THÔNG SỐ VẬT LÝ TỪ BÀI BÁO FLIX-WALKER (Jiang et al. 2025) ---
    L = 0.05            # Leg Length: 50 mm = 0.05 m
    b = 0.02            # Leg Width: 20 mm = 0.02 m
    h = 0.003           # Leg Thickness: 3 mm = 0.003 m
    
    # Vật liệu: Ultimaker TPU 95A
    E = 26e6            # Young's Modulus: 26 MPa
    nu = 0.48           # Poisson's ratio
    G = E / (2 * (1 + nu)) # Shear Modulus

    # --- 2. TÍNH ĐẶC TRƯNG TIẾT DIỆN ---
    # Hằng số xoắn Saint-Venant (It) cho tiết diện chữ nhật
    a_long = b
    b_short = h
    It = a_long * b_short**3 * (1/3 - 0.21 * (b_short/a_long) * (1 - (b_short/a_long)**4 / 12))
    
    # Hằng số vênh (Iw) cho tiết diện chữ nhật đặc
    Iw = (b**3 * h**3) / 144  
    
    # Hệ số lambda (đặc trưng tắt dần của hiệu ứng Vlasov)
    lam = np.sqrt((G * It) / (E * Iw))

    print(f"--- THÔNG SỐ ĐẦU VÀO ---")
    print(f"Kích thước: L={L*1000}mm, W={b*1000}mm, T={h*1000}mm")
    print(f"Vật liệu TPU 95A: E={E/1e6} MPa, G={G/1e6:.2f} MPa")
    print(f"Hằng số hình học: It={It:.2e} m4, Iw={Iw:.2e} m6")
    print(f"Lambda Vlasov: {lam:.2f}")

    # --- 3. BỘ GIẢI SỐ (BVP SOLVER) ---
    def fun(x, y):
        # y = [phi, phi', phi'', phi''']
        return np.vstack((y[1], y[2], y[3], (lam**2) * y[2]))

    def bc(ya, yb, Mt):
        # Tại ngàm (x=0): phi=0, phi'=0 (Ngàm cứng chống vênh)
        res_a = np.array([ya[0], ya[1]])
        
        # Tại đầu tự do (x=L):
        # 1. Bimoment = 0 => phi'' = 0
        # 2. Torque cân bằng => T = G*It*phi' - E*Iw*phi''' = Mt
        res_b = np.array([yb[2], G*It*yb[1] - E*Iw*yb[3] - Mt])
        return np.hstack((res_a, res_b))

    # --- 4. TÍNH TOÁN PRBM (n=3) VỚI ĐA DẠNG TẢI ---
    n_segments = 3
    l_seg = L / n_segments
    
    # ĐA DẠNG TẢI TRỌNG: 1000 trường hợp tải từ 0.1 Nmm đến 0.5 Nmm
    load_steps = np.linspace(1e-4, 5e-4, 1000)  # N.m (0.1 Nmm to 0.5 Nmm)

    numerator_k = np.zeros(n_segments)
    denominator_k = np.zeros(n_segments)
    
    x_plot = np.linspace(0, L, 100)
    
    print(f"\nĐang xử lý {len(load_steps)} trường hợp tải...")
    
    for idx, Mt in enumerate(load_steps):
        def bc_wrapper(ya, yb):
            return bc(ya, yb, Mt)

        y_guess = np.zeros((4, x_plot.size))
        
        # Giải phương trình
        sol = solve_bvp(fun, bc_wrapper, x_plot, y_guess, tol=1e-5)
        
        if sol.success:
            # Lấy giá trị góc xoắn tại các điểm nút (L/3, 2L/3, L)
            phi_nodes = [sol.sol(i * l_seg)[0] for i in range(1, n_segments + 1)]
            
            # Tính delta_phi (góc biến dạng tương đối)
            delta_phi = np.zeros(n_segments)
            prev_phi = 0
            for i in range(n_segments):
                delta_phi[i] = phi_nodes[i] - prev_phi
                prev_phi = phi_nodes[i]
                
                # Tích lũy cho tính toán k (Least Squares)
                numerator_k[i] += Mt * delta_phi[i]
                denominator_k[i] += delta_phi[i]**2
        else:
            print(f"Solver không hội tụ tại bước tải {idx}: Mt={Mt}")

    # --- 5. KẾT QUẢ ---
    k_prbm = numerator_k / (denominator_k + 1e-20)
    
    return k_prbm

k_vals_diverse = calculate_flix_walker_stiffness_diversified_loads()
print(f"k1 (Gần ngàm): {k_vals_diverse[0]:.6f} Nm/rad ({k_vals_diverse[0]*1000:.3f} Nmm/rad)")
print(f"k2 (Giữa):     {k_vals_diverse[1]:.6f} Nm/rad ({k_vals_diverse[1]*1000:.3f} Nmm/rad)")
print(f"k3 (Đầu tip):  {k_vals_diverse[2]:.6f} Nm/rad ({k_vals_diverse[2]*1000:.3f} Nmm/rad)")
print(f"Tỷ lệ k1/k2: {k_vals_diverse[0]/k_vals_diverse[1]:.2f}")