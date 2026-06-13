import pandas as pd
import numpy as np
df = pd.read_csv('datalog.csv')
df_armed = df[(df['t'] > 4.10) & (df['throttleFwdRight commanded'] > 0.15)]
if len(df_armed) > 0:
    t = df_armed['t'].values
    p = df_armed['p'].values
    crossings = np.where(np.diff(np.sign(p)))[0]
    if len(crossings) > 1:
        periods = np.diff(t[crossings]) * 2
        freq = 1.0 / np.mean(periods)
        print(f"Roll Oscillation freq: {freq:.1f} Hz")
