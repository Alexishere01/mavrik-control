import pandas as pd
import numpy as np

def quat_to_euler(e0, ex, ey, ez):
    phi = np.arctan2(2 * (e0 * ex + ey * ez), 1 - 2 * (ex**2 + ey**2))
    theta = np.arcsin(np.clip(2 * (e0 * ey - ez * ex), -1.0, 1.0))
    psi = np.arctan2(2 * (e0 * ez + ex * ey), 1 - 2 * (ey**2 + ez**2))
    return phi * 180/np.pi, theta * 180/np.pi, psi * 180/np.pi

try:
    df = pd.read_csv('datalog.csv')
    df_armed = df[df['throttleFwdRight commanded'] > 0.05]
    if len(df_armed) == 0:
        print("No armed flight data.")
    else:
        roll, pitch, yaw = quat_to_euler(df_armed['e0'], df_armed['ex'], df_armed['ey'], df_armed['ez'])
        df_armed = df_armed.copy()
        df_armed['roll'] = roll
        
        # Find a moment of significant roll
        idx = df_armed['roll'].idxmax()
        row = df_armed.loc[idx]
        print(f"Max roll: {row['roll']:.1f} deg at t={row['t']:.1f}s")
        
        fr = row['throttleFwdRight commanded']
        fl = row['throttleFwdLeft commanded']
        ar = row['throttleAftRight commanded']
        al = row['throttleAftLeft commanded']
        
        right_thr = fr + ar
        left_thr = fl + al
        
        print(f"Right Thrust: {right_thr:.3f} (FR={fr:.3f}, AR={ar:.3f})")
        print(f"Left Thrust:  {left_thr:.3f} (FL={fl:.3f}, AL={al:.3f})")
        
        if row['roll'] > 0:
            # Drone is rolled RIGHT. It should be pushing RIGHT up (Right Thrust > Left Thrust).
            if right_thr > left_thr:
                print("Controller is pushing RIGHT UP (correcting roll RIGHT). MAPPING OK.")
            else:
                print("Controller is pushing LEFT UP (exacerbating roll RIGHT). MAPPING REVERSED!")
        else:
            if left_thr > right_thr:
                print("Controller is pushing LEFT UP (correcting roll LEFT). MAPPING OK.")
            else:
                print("Controller is pushing RIGHT UP (exacerbating roll LEFT). MAPPING REVERSED!")

except Exception as e:
    print(e)
