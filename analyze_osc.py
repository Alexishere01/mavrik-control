import pandas as pd
df = pd.read_csv('datalog.csv')
df_armed = df[(df['t'] > 3.7) & (df['throttleFwdRight commanded'] > 0.15)].copy()
if len(df_armed) > 0:
    print(f"Roll rate std: {df_armed['p'].std() * 180/3.14159:.1f} deg/s")
    print(f"Pitch rate std: {df_armed['q'].std() * 180/3.14159:.1f} deg/s")
