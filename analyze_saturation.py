import pandas as pd
import numpy as np

df = pd.read_csv('datalog.csv')
df_armed = df[(df['t'] > 4.7) & (df['throttleFwdRight commanded'] > 0.15)]
if len(df_armed) > 0:
    print(f"Max roll_diff: {df_armed['roll_diff'].max():.3f}")
    print(f"Min roll_diff: {df_armed['roll_diff'].min():.3f}")
    print(f"Max pitch_fwd: {df_armed['pitch_diff_fwd'].max():.3f}")
    print(f"Min pitch_fwd: {df_armed['pitch_diff_fwd'].min():.3f}")
