import pandas as pd
import numpy as np

df = pd.read_csv('datalog.csv')
df_armed = df[(df['t'] > 4.7) & (df['throttleFwdRight commanded'] > 0.15)].copy()

if len(df_armed) > 0:
    t = df_armed['t'].values
    p = df_armed['p'].values * 180/np.pi
    
    crossings = np.where(np.diff(np.sign(p)))[0]
    if len(crossings) > 1:
        periods = np.diff(t[crossings]) * 2
        freq = 1.0 / np.mean(periods)
        print(f"Roll Oscillation frequency: {freq:.1f} Hz")
        print(f"Max roll rate: {np.max(np.abs(p)):.1f} deg/s")
    else:
        print("Not enough crossings.")
