import pandas as pd
df = pd.read_csv('bridge_diag.csv')
print("Max AP PWMs:", df[['ap_pwm1', 'ap_pwm2', 'ap_pwm3', 'ap_pwm4']].max().to_dict())
print("Max MAVRIK thr:", df[['thrFR', 'thrFL', 'thrAR', 'thrAL']].max().to_dict())
