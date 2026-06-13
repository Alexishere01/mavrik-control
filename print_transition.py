import pandas as pd
import numpy as np

def quat_to_euler(e0, ex, ey, ez):
    phi = np.arctan2(2 * (e0 * ex + ey * ez), 1 - 2 * (ex**2 + ey**2))
    theta = np.arcsin(np.clip(2 * (e0 * ey - ez * ex), -1.0, 1.0))
    psi = np.arctan2(2 * (e0 * ez + ex * ey), 1 - 2 * (ey**2 + ez**2))
    return phi * 180/np.pi, theta * 180/np.pi, psi * 180/np.pi

df = pd.read_csv('datalog.csv')
df['roll'], df['pitch'], df['yaw'] = quat_to_euler(
    df['e0'], df['ex'], df['ey'], df['ez']
)

# Look for when they rolled out (roll > 10.0 degrees)
roll_start = df[abs(df['roll']) > 10.0]
if len(roll_start) > 0:
    t_roll = roll_start['t'].iloc[0]
    print(f"Roll > 10deg detected at t = {t_roll:.2f}s")
    
    # Print from t = 18.0 to 24.0
    df_window = df[(df['t'] > 18.0) & (df['t'] < 24.0)]
    print(df_window[['t', 'roll', 'pitch', 'yaw', 'throttleFwdRight commanded', 'throttleFwdLeft commanded', 'throttleAftRight commanded', 'throttleAftLeft commanded']].to_string(index=False))
else:
    print("No rollout detected.")
