import pandas as pd
df = pd.read_csv('bridge_diag.csv')
armed_rows = df[df['armed'] == 1]
if len(armed_rows) > 0:
    first_armed_idx = armed_rows.index[0]
    flight_df = df.iloc[first_armed_idx :: 20] # Downsample by 20 for readability
    print("Flight profile (downsampled):")
    # zf is the position coordinate in down (so -zf is altitude in meters!)
    print(flight_df[['sim_t', 'roll_deg', 'pitch_deg', 'yaw_deg', 'zf', 'ap_pwm1', 'ap_pwm2', 'ap_pwm3', 'ap_pwm4']])
