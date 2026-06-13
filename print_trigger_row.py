import pandas as pd
df = pd.read_csv('bridge_diag.csv')
armed_rows = df[df['armed'] == 1]
if len(armed_rows) > 0:
    idx = armed_rows.index[0]
    print("Row where armed became 1:")
    print(df.iloc[idx][['sim_t', 'armed', 'roll_deg', 'pitch_deg', 'yaw_deg', 'ap_pwm1', 'ap_pwm2', 'ap_pwm3', 'ap_pwm4']])
    print("\nRow before:")
    print(df.iloc[idx-1][['sim_t', 'armed', 'roll_deg', 'pitch_deg', 'yaw_deg', 'ap_pwm1', 'ap_pwm2', 'ap_pwm3', 'ap_pwm4']])
else:
    print("Armed is never 1!")
