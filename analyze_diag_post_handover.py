import pandas as pd
df = pd.read_csv('bridge_diag.csv')
# Let's find rows where armed changes to 1 (which represents has_armed = True in the log!)
armed_rows = df[df['armed'] == 1]
if len(armed_rows) > 0:
    first_armed_idx = armed_rows.index[0]
    start_idx = max(0, first_armed_idx - 5)
    end_idx = min(len(df), first_armed_idx + 80)
    print("\n Telemetry AFTER handover:")
    print(df.iloc[start_idx:end_idx][['sim_t', 'roll_deg', 'pitch_deg', 'armed', 'ap_pwm1', 'ap_pwm2', 'ap_pwm3', 'ap_pwm4', 'thrFR', 'thrFL', 'thrAR', 'thrAL']].to_string())
else:
    print("Handover (armed == 1) never occurred in this bridge_diag.csv!")
