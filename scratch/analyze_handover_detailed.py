import pandas as pd

df = pd.read_csv("bridge_diag.csv")
pd.set_option('display.max_columns', 25)
pd.set_option('display.max_rows', 150)
pd.set_option('display.width', 1000)

print(df.iloc[1060:1180][['sim_t', 'alt_ft', 'roll_deg', 'pitch_deg', 'yaw_deg', 'p_rads', 'q_rads', 'r_rads', 'thrFR', 'thrFL', 'thrAR', 'thrAL', 'ap_has_armed', 'ap_pwm1', 'ap_pwm2', 'ap_pwm3', 'ap_pwm4']].to_string())
