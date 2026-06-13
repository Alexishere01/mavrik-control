import pandas as pd
import numpy as np

try:
    df = pd.read_csv('datalog.csv')
    df_armed = df[df['throttleFwdRight commanded'] > 0.05]
    if len(df_armed) == 0:
        print("No armed flight data found.")
    else:
        print(f"Flight time: {df_armed['t'].max() - df_armed['t'].min():.1f}s")
        
        def quat_to_euler(e0, ex, ey, ez):
            phi = np.arctan2(2 * (e0 * ex + ey * ez), 1 - 2 * (ex**2 + ey**2))
            theta = np.arcsin(np.clip(2 * (e0 * ey - ez * ex), -1.0, 1.0))
            psi = np.arctan2(2 * (e0 * ez + ex * ey), 1 - 2 * (ey**2 + ez**2))
            return phi * 180/np.pi, theta * 180/np.pi, psi * 180/np.pi

        roll, pitch, yaw = quat_to_euler(df_armed['e0'], df_armed['ex'], df_armed['ey'], df_armed['ez'])
        
        print(f"Pitch: mean = {pitch.mean():.2f} deg, std = {pitch.std():.2f} deg")
        print(f"Yaw:   std = {yaw.std():.2f} deg, rate std = {(df_armed['r'] * 180/np.pi).std():.2f} deg/s")
        print(f"Roll:  std = {roll.std():.2f} deg")
        
        # Check arms dancing
        flapsR_std = df_armed['flapsRight commanded'].std()
        flapsL_std = df_armed['flapsLeft commanded'].std()
        print(f"Flaps R std: {flapsR_std:.4f}, Flaps L std: {flapsL_std:.4f}")
        
except Exception as e:
    print("Error analyzing datalog:", e)

