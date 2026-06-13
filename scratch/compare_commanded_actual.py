import pandas as pd

df = pd.read_csv('/Users/alex/Library/Containers/com.isaacmarovitz.Whisky/Bottles/37084628-4FFA-403A-BD20-3D881F401401/drive_c/zWindowsDeployment/datalog.csv')
df_armed = df[df['t'] > 21.0].copy()

# Downsample for readability
cols = [
    't', 'pitch', 'q', 
    'throttleFwdRight commanded', 'throttleFwdRight actual',
    'throttleAftRight commanded', 'throttleAftRight actual',
    'elevator commanded', 'elevator actual'
]

# Note: pitch in datalog is not degrees, let's print euler pitch as well
import numpy as np
def quat_to_pitch(e0, ex, ey, ez):
    sinp = 2.0 * (e0 * ey - ez * ex)
    return np.arcsin(np.clip(sinp, -1.0, 1.0)) * 180.0 / np.pi

df_armed['pitch_deg'] = quat_to_pitch(df_armed['e0'], df_armed['ex'], df_armed['ey'], df_armed['ez'])

print(f"{'t':<6} | {'pitch':<8} | {'thrFR_cmd':<9} | {'thrFR_act':<9} | {'thrAR_cmd':<9} | {'thrAR_act':<9} | {'ele_cmd':<8} | {'ele_act':<8}")
print("="*100)
for idx, r in df_armed.iloc[::10].iterrows():
    print(f"{r['t']:6.2f} | {r['pitch_deg']:8.2f} | {r['throttleFwdRight commanded']:9.4f} | {r['throttleFwdRight actual']:9.4f} | {r['throttleAftRight commanded']:9.4f} | {r['throttleAftRight actual']:9.4f} | {r['elevator commanded']:8.2f} | {r['elevator actual']:8.2f}")
