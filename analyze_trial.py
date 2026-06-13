import pandas as pd
df = pd.read_csv('tune_logs/trial_0095_s20.5.csv')
print("Columns in file:")
print(df.columns)

print("\nTelemetry from t=4.5s to t=8s:")
print(df[(df['sim_t'] > 4.5) & (df['sim_t'] < 8.0)][['sim_t', 'roll_deg', 'pitch_deg', 'p_rads', 'thrFR', 'thrFL']].head(30))
