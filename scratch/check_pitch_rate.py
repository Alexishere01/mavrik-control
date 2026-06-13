import pandas as pd
import numpy as np

df = pd.read_csv('/Users/alex/Library/Containers/com.isaacmarovitz.Whisky/Bottles/37084628-4FFA-403A-BD20-3D881F401401/drive_c/zWindowsDeployment/bridge_diag.csv')
df_active = df[df['sim_t'] > 22.0].copy()

dt = df_active['sim_t'].diff()
dpitch_dt = np.radians(df_active['pitch_deg'].diff()) / dt

print(f"{'sim_t':<6} | {'pitch_deg':<10} | {'q_rads (gyro)':<14} | {'dpitch/dt':<12}")
print("="*60)
for idx, r in df_active.iloc[::2].iterrows():
    if idx == df_active.index[0]:
        continue
    dp = dpitch_dt.loc[idx]
    print(f"{r['sim_t']:6.2f} | {r['pitch_deg']:10.2f} | {r['q_rads']:14.4f} | {dp:12.4f}")
