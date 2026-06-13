import pandas as pd
df = pd.read_csv('bridge_diag.csv')
armed_rows = df[df['ap_has_armed'] == 1]
if len(armed_rows) > 0:
    first_armed_idx = armed_rows.index[0]
    start_idx = max(0, first_armed_idx - 10)
    end_idx = min(len(df), first_armed_idx + 60)
    print("\n Handover transition telemetry:")
    print(df.iloc[start_idx:end_idx][['sim_t', 'roll_deg', 'pitch_deg', 'ap_has_armed', 'ap_pwm1', 'ap_pwm2', 'ap_pwm3', 'ap_pwm4', 'thrFR', 'thrFL', 'thrAR', 'thrAL']].to_string())
else:
    print("Never armed in bridge_diag.csv!")
