import pandas as pd
import numpy as np

df = pd.read_csv('/Users/alex/Library/Containers/com.isaacmarovitz.Whisky/Bottles/37084628-4FFA-403A-BD20-3D881F401401/drive_c/zWindowsDeployment/bridge_diag.csv')

# Filter for armed period
df_armed = df[df['ap_has_armed'] == 1].copy()

print(f"{'sim_t':<6} | {'pitch[°]':<8} | {'roll[°]':<8} | {'u_body':<8} | {'w_body':<8} | {'vn':<8} | {'vd':<8} | {'alt_ft':<8} | {'ele':<8}")
print("="*90)
for idx, r in df_armed.iloc[::2].iterrows():
    # We estimate vn, vd using body_to_earth or just look at them if they are in the file.
    # Actually, let's check what columns we have:
    # "wall_clock", "sim_t", "alt_ft", "roll_deg", "pitch_deg", "yaw_deg", "u_fps", "v_fps", "w_fps", "p_rads", "q_rads", "r_rads"
    print(f"{r['sim_t']:6.2f} | {r['pitch_deg']:8.2f} | {r['roll_deg']:8.2f} | {r['u_fps']:8.2f} | {r['w_fps']:8.2f} | {r['alt_ft']:8.2f} | {r['ele']:8.2f}")
