import pandas as pd
import math

df = pd.read_csv('bridge_diag.csv')
armed_rows = df[df['armed'] == 1]
if len(armed_rows) > 0:
    first_armed_idx = armed_rows.index[0]
    t_handover = df.iloc[first_armed_idx]['sim_t']
    print(f"Handover occurred at sim_t = {t_handover:.4f}s")
    
    # Print telemetry from handover up to the end (or next 150 rows)
    transition_df = df.iloc[first_armed_idx : min(len(df), first_armed_idx + 150)]
    print("\n Telemetry around handover and subsequent steps:")
    print(transition_df[['sim_t', 'roll_deg', 'pitch_deg', 'yaw_deg', 'ap_pwm1', 'ap_pwm2', 'ap_pwm3', 'ap_pwm4', 'thrFR', 'thrFL', 'thrAR', 'thrAL']].to_string())
else:
    print("No handover/arming detected in bridge_diag.csv!")
    print(df.tail(30)[['sim_t', 'roll_deg', 'pitch_deg', 'yaw_deg', 'ap_pwm1', 'ap_pwm2', 'ap_pwm3', 'ap_pwm4']].to_string())
