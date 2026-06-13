import pandas as pd
df = pd.read_csv('tune_logs/trial_0096_s18.4.csv')
arm_t = df[df['ap_has_armed'] == 1]['sim_t'].iloc[0]
df_active = df[(df['sim_t'] >= arm_t + 4.5) & (df['sim_t'] <= arm_t + 7.5)]
print(df_active[['sim_t', 'roll_deg', 'pitch_deg', 'yaw_deg', 'p_rads', 'q_rads', 'thrFR', 'thrAR']].iloc[::10])
