import pandas as pd
import numpy as np

df = pd.read_csv('datalog.csv')

def quat_to_euler(e0, ex, ey, ez):
    phi = np.arctan2(2 * (e0 * ex + ey * ez), 1 - 2 * (ex**2 + ey**2))
    theta = np.arcsin(np.clip(2 * (e0 * ey - ez * ex), -1.0, 1.0))
    psi = np.arctan2(2 * (e0 * ez + ex * ey), 1 - 2 * (ey**2 + ez**2))
    return phi * 180/np.pi, theta * 180/np.pi, psi * 180/np.pi

roll, pitch, yaw = quat_to_euler(df['e0'], df['ex'], df['ey'], df['ez'])
df['roll'] = roll
df['pitch'] = pitch
df['yaw'] = yaw

print(f"Total time logged: {df['t'].max():.1f}s")

# Let's see if the motors were ever above hover throttle (e.g. 0.2)
max_thr = df[['throttleFwdRight commanded', 'throttleFwdLeft commanded', 'throttleAftRight commanded', 'throttleAftLeft commanded']].max().max()
print(f"Max throttle commanded: {max_thr:.3f}")

if max_thr > 0.05:
    df_armed = df[df['throttleFwdRight commanded'] > 0.05]
    print(f"Flight time: {df_armed['t'].max() - df_armed['t'].min():.1f}s")
    
    pitch_mean = df_armed['pitch'].mean()
    pitch_std = df_armed['pitch'].std()
    yaw_std = df_armed['yaw'].std()
    yaw_rate_std = df_armed['r'].std() * 180/np.pi
    
    print(f"Pitch: mean = {pitch_mean:.2f} deg, std = {pitch_std:.2f} deg")
    print(f"Yaw:   std = {yaw_std:.2f} deg, rate std = {yaw_rate_std:.2f} deg/s")
else:
    print("Motors never spooled up. Did you arm?")

