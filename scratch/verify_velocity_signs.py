import pandas as pd
import numpy as np

# Load bridge_diag.csv
df = pd.read_csv('/Users/alex/Library/Containers/com.isaacmarovitz.Whisky/Bottles/37084628-4FFA-403A-BD20-3D881F401401/drive_c/zWindowsDeployment/bridge_diag.csv')

# Filter for active flight (sim_t > 21.0)
df = df[df['sim_t'] > 21.0].copy()

# Compute position derivatives using finite differences
dt = df['sim_t'].diff()
d_pn = (df['u_fps'] * 0.3048).diff() # just a placeholder, let's use the actual position fields
# Wait! In bridge_diag.csv, do we have positions?
# Let's check the columns in bridge_diag.csv.
# "wall_clock", "sim_t", "alt_ft", "roll_deg", "pitch_deg", "yaw_deg", "u_fps", "v_fps", "w_fps", "p_rads", "q_rads", "r_rads"
# Wait! bridge_diag.csv does not have xf, yf, zf, but we can load datalog.csv which has everything!
# Let's load datalog.csv.
df_data = pd.read_csv('/Users/alex/Library/Containers/com.isaacmarovitz.Whisky/Bottles/37084628-4FFA-403A-BD20-3D881F401401/drive_c/zWindowsDeployment/datalog.csv')
df_data = df_data[df_data['t'] > 21.0].copy()

dt = df_data['t'].diff()
# Position in NED RHS: pn = xf, pe = -yf, pd = zf
pn = df_data['xf'] * 0.3048
pe = -df_data['yf'] * 0.3048
pd_coord = df_data['zf'] * 0.3048

# Numerical derivatives (m/s)
dpn_dt = pn.diff() / dt
dpe_dt = pe.diff() / dt
dpd_dt = pd_coord.diff() / dt

# MAVRIK body velocities (m/s)
u_m = df_data['u'] * 0.3048
v_m = df_data['v'] * 0.3048
w_m = df_data['w'] * 0.3048

def body_to_earth(v_vec, q):
    x, y, z = v_vec
    e0, ex, ey, ez = q
    T0 = x * ex + y * ey + z * ez
    T1 = x * e0 - y * ez + z * ey
    T2 = x * ez + y * e0 - z * ex
    T3 = y * ex - x * ey + z * e0
    return (e0 * T1 + ex * T0 + ey * T3 - ez * T2,
            e0 * T2 - ex * T3 + ey * T0 + ez * T1,
            e0 * T3 + ex * T2 - ey * T1 + ez * T0)

# Quaternion Option F: (e0, -ex, ey, -ez)
# Let's compute rotated earth velocities for different sign combinations of (u, v, w)
# Combination 1: (u, -v, -w)  -- current bridge
# Combination 2: (u, -v, w)
# Combination 3: (u, v, -w)
# Combination 4: (u, v, w)
# Combination 5: (-u, v, w) etc.

results = []
for idx in df_data.index:
    if idx == df_data.index[0]:
        continue
    t_val = df_data.loc[idx, 't']
    q_ned = (df_data.loc[idx, 'e0'], -df_data.loc[idx, 'ex'], df_data.loc[idx, 'ey'], -df_data.loc[idx, 'ez'])
    
    # Numerical derivatives
    vn_num = dpn_dt.loc[idx]
    ve_num = dpe_dt.loc[idx]
    vd_num = dpd_dt.loc[idx]
    
    # Check if dt is valid
    if not np.isfinite(vn_num) or dt.loc[idx] < 0.001:
        continue
        
    u_b = u_m.loc[idx]
    v_b = v_m.loc[idx]
    w_b = w_m.loc[idx]
    
    # Rotated
    v1 = body_to_earth((u_b, -v_b, -w_b), q_ned)
    v2 = body_to_earth((u_b, -v_b, w_b), q_ned)
    v3 = body_to_earth((u_b, v_b, -w_b), q_ned)
    v4 = body_to_earth((u_b, v_b, w_b), q_ned)
    
    results.append({
        't': t_val,
        'vn_num': vn_num, 've_num': ve_num, 'vd_num': vd_num,
        'vd_1': v1[2], 'vd_2': v2[2], 'vd_3': v3[2], 'vd_4': v4[2],
        'vn_1': v1[0], 'vn_2': v2[0], 'vn_3': v3[0], 'vn_4': v4[0],
        've_1': v1[1], 've_2': v2[1], 've_3': v3[1], 've_4': v4[1],
    })

df_res = pd.DataFrame(results)

print("Mean Absolute Errors (MAE) for vd (vertical velocity):")
print(f"  (u, -v, -w) [current]: {np.mean(np.abs(df_res['vd_1'] - df_res['vd_num'])):.3f} m/s")
print(f"  (u, -v,  w):           {np.mean(np.abs(df_res['vd_2'] - df_res['vd_num'])):.3f} m/s")
print(f"  (u,  v, -w):           {np.mean(np.abs(df_res['vd_3'] - df_res['vd_num'])):.3f} m/s")
print(f"  (u,  v,  w):           {np.mean(np.abs(df_res['vd_4'] - df_res['vd_num'])):.3f} m/s")

print("\nMean Absolute Errors (MAE) for ve (lateral velocity):")
print(f"  (u, -v, -w) [current]: {np.mean(np.abs(df_res['ve_1'] - df_res['ve_num'])):.3f} m/s")
print(f"  (u, -v,  w):           {np.mean(np.abs(df_res['ve_2'] - df_res['ve_num'])):.3f} m/s")
print(f"  (u,  v, -w):           {np.mean(np.abs(df_res['ve_3'] - df_res['ve_num'])):.3f} m/s")
print(f"  (u,  v,  w):           {np.mean(np.abs(df_res['ve_4'] - df_res['ve_num'])):.3f} m/s")

print("\nMean Absolute Errors (MAE) for vn (forward velocity):")
print(f"  (u, -v, -w) [current]: {np.mean(np.abs(df_res['vn_1'] - df_res['vn_num'])):.3f} m/s")
print(f"  (u, -v,  w):           {np.mean(np.abs(df_res['vn_2'] - df_res['vn_num'])):.3f} m/s")
print(f"  (u,  v, -w):           {np.mean(np.abs(df_res['vn_3'] - df_res['vn_num'])):.3f} m/s")
print(f"  (u,  v,  w):           {np.mean(np.abs(df_res['vn_4'] - df_res['vn_num'])):.3f} m/s")

print("\nSample values around sim_t = 22.0:")
sample = df_res[(df_res['t'] >= 21.9) & (df_res['t'] <= 22.1)].iloc[0]
print(f"Numerical: vn={sample['vn_num']:.2f}, ve={sample['ve_num']:.2f}, vd={sample['vd_num']:.2f}")
print(f"Current:   vn={sample['vn_1']:.2f}, ve={sample['ve_1']:.2f}, vd={sample['vd_1']:.2f}")
print(f"(u,v,w):   vn={sample['vn_4']:.2f}, ve={sample['ve_4']:.2f}, vd={sample['vd_4']:.2f}")
print(f"(u,-v,w):  vn={sample['vn_2']:.2f}, ve={sample['ve_2']:.2f}, vd={sample['vd_2']:.2f}")
