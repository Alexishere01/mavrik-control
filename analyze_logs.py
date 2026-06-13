import pandas as pd
import numpy as np

# Read bridge_diag.csv
df = pd.read_csv('bridge_diag.csv')

# Only consider data after arming (ap_has_armed == True or motor_active)
if 'ap_has_armed' in df.columns:
    df_armed = df[df['ap_has_armed'] == True]
else:
    df_armed = df

if len(df_armed) < 50:
    print("Not enough armed data in bridge_diag.csv")
else:
    # Let's look at the last 10 seconds of hover
    df_stable = df_armed[df_armed['sim_t'] > df_armed['sim_t'].max() - 10]
    
    print(f"--- Analysis over last 10s of armed flight (t={df_stable['sim_t'].min():.1f} to {df_stable['sim_t'].max():.1f}) ---")
    
    pitch_mean = df_stable['pitch_deg'].mean()
    pitch_std = df_stable['pitch_deg'].std()
    yaw_std = df_stable['yaw_deg'].std()
    
    # Calculate yaw rate std to see oscillations
    yaw_rate_std = df_stable['r_rads'].std() * (180/np.pi)
    
    print(f"Pitch: mean = {pitch_mean:.2f} deg, std = {pitch_std:.2f} deg")
    print(f"Yaw:   std = {yaw_std:.2f} deg, rate std = {yaw_rate_std:.2f} deg/s")
    
    if pitch_mean < -0.5 or pitch_mean > 0.5:
        print("Pitch mean is offset -> I-term could help eliminate steady-state error.")
    if yaw_rate_std > 2.0:
        print("Yaw rate is oscillating > 2 deg/s -> Yaw tuning might be too aggressive (high P/D) or lacking damping.")

