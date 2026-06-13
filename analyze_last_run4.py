import pandas as pd
df = pd.read_csv('bridge_diag.csv')
armed_rows = df[df['armed'] == 1]
if len(armed_rows) > 0:
    first_armed_idx = armed_rows.index[0]
    print(f"Total armed rows: {len(df) - first_armed_idx}")
    
    # Print telemetry from 100 rows after handover up to the end
    end_df = df.iloc[first_armed_idx + 150 : first_armed_idx + 350]
    print("\n Telemetry further in flight:")
    print(end_df[['sim_t', 'roll_deg', 'pitch_deg', 'yaw_deg', 'ap_pwm1', 'ap_pwm2', 'ap_pwm3', 'ap_pwm4', 'thrFR', 'thrFL', 'thrAR', 'thrAL']].to_string())
