import pandas as pd
df = pd.read_csv('bridge_diag.csv')
armed_rows = df[df['armed'] == 1]
if len(armed_rows) > 0:
    first_armed_idx = armed_rows.index[0]
    
    # We want to look around sim_t = 15s to 25s (rows 700 to 1200)
    mid_df = df.iloc[first_armed_idx + 400 : first_armed_idx + 800]
    print("\n Telemetry from 15s to 25s:")
    print(mid_df[['sim_t', 'roll_deg', 'pitch_deg', 'yaw_deg', 'ap_pwm1', 'ap_pwm2', 'ap_pwm3', 'ap_pwm4', 'thrFR', 'thrFL', 'thrAR', 'thrAL']].to_string())
