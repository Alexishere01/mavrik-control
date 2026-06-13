import pandas as pd
df = pd.read_csv('bridge_diag.csv')
print(f"Total rows in bridge_diag.csv: {len(df)}")
print(f"Max sim_t: {df['sim_t'].max()}")
print("\nTelemetry from the absolute end:")
print(df.tail(30)[['sim_t', 'roll_deg', 'pitch_deg', 'yaw_deg', 'ap_pwm1', 'ap_pwm2', 'ap_pwm3', 'ap_pwm4', 'thrFR', 'thrFL', 'thrAR', 'thrAL']])
