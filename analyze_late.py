import pandas as pd
df = pd.read_csv('datalog.csv')
df_armed = df[(df['t'] > 3.0) & (df['throttleFwdRight commanded'] > 0.15)].copy()
if len(df_armed) > 0:
    print(f"Time range of active flight: {df_armed['t'].min():.2f} to {df_armed['t'].max():.2f}")
    print(f"Max throttle: {df_armed['throttleFwdRight commanded'].max():.3f}")
    
    # Calculate std only when throttle is stable-ish (holding hover)
    df_hover = df_armed[df_armed['throttleFwdRight commanded'] < 0.8]
    if len(df_hover) > 0:
        print(f"Roll rate std during hover: {df_hover['p'].std() * 180/3.14159:.1f} deg/s")
        print(f"Pitch rate std during hover: {df_hover['q'].std() * 180/3.14159:.1f} deg/s")
    
    print("\nSample of commands during active flight:")
    print(df_armed[['t', 'roll', 'pitch', 'yaw', 'throttleFwdRight commanded']].iloc[::50].head())
else:
    print("No active flight (throttle > 0.15) found after arming.")
