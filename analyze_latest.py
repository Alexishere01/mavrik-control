import pandas as pd
df = pd.read_csv('datalog.csv')
print(f"Total time: {df['t'].max()}")
print(f"Max roll: {df['roll'].max() * 180/3.14159:.1f}, Min roll: {df['roll'].min() * 180/3.14159:.1f}")
print(f"Max pitch: {df['pitch'].max() * 180/3.14159:.1f}, Min pitch: {df['pitch'].min() * 180/3.14159:.1f}")
df_armed = df[df['t'] > 3.0]
if len(df_armed) > 0:
    print(f"Armed max throttle: {df_armed['throttleFwdRight commanded'].max():.3f}")
    
    # Let's find when they rolled out
    roll_start = df_armed[abs(df_armed['roll']) > 0.5]
    if len(roll_start) > 0:
        print(f"Roll started at t = {roll_start['t'].iloc[0]:.2f}")
        crash = df_armed[df_armed['t'] > roll_start['t'].iloc[0]]
        print(f"Roll rate std after rollout: {crash['p'].std() * 180/3.14159:.1f} deg/s")
