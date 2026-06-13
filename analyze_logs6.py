import pandas as pd
df = pd.read_csv('datalog.csv')
if df['throttleFwdRight commanded'].max() > 0.05:
    df_armed = df[df['throttleFwdRight commanded'] > 0.05]
    print(f"Max U (forward speed): {df_armed['u'].max():.1f} ft/s")
