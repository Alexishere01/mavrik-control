import pandas as pd
df = pd.read_csv('tune_logs/trial_0095_s20.5.csv')
print(df[df['ap_has_armed'] == 1][['sim_t', 'roll_deg', 'pitch_deg', 'p_rads', 'q_rads', 'thrFR', 'thrFL', 'thrAR', 'thrAL']].head(40))
