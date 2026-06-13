import pandas as pd
# find the latest trial 95 log
import glob
files = sorted(glob.glob('tune_logs/trial_0095*.csv'))
if not files:
    print("Log not found")
else:
    df = pd.read_csv(files[-1])
    # Print the maneuver
    post = df[df['t'] > 3.0] # armed after ~3s
    print("Time, Roll, Pitch, Roll Command")
    print(post[['t', 'roll_deg', 'pitch_deg', 'thrFwdRight']].iloc[::25].head(20).to_string())
