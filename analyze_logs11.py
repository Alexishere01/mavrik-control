import pandas as pd
import numpy as np

def quat_to_euler(e0, ex, ey, ez):
    phi = np.arctan2(2 * (e0 * ex + ey * ez), 1 - 2 * (ex**2 + ey**2))
    return phi * 180/np.pi

df = pd.read_csv('datalog.csv')
df_armed = df[df['throttleFwdRight commanded'] > 0.05].copy()
if len(df_armed) > 0:
    df_armed['roll'] = quat_to_euler(df_armed['e0'], df_armed['ex'], df_armed['ey'], df_armed['ez'])
    # Print the first 10 seconds of roll
    start_t = df_armed['t'].min()
    sample = df_armed[df_armed['t'] <= start_t + 5]
    print(sample[['t', 'roll', 'throttleFwdRight commanded', 'throttleFwdLeft commanded', 'throttleAftRight commanded', 'throttleAftLeft commanded']].head(20).to_string(index=False))
