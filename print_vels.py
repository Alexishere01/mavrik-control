import pandas as pd
df = pd.read_csv('bridge_diag.csv')
armed_rows = df[df['armed'] == 1]
if len(armed_rows) > 0:
    first_armed_idx = armed_rows.index[0]
    flight_df = df.iloc[first_armed_idx :: 40]
    print("Velocity profile:")
    for idx, row in flight_df.iterrows():
        print(f"sim_t={row['sim_t']:5.2f}s | yaw={row['yaw_deg']:6.2f}° | roll={row['roll_deg']:6.2f}° | u={row['u_fps']:5.1f} fps | v={row['v_fps']:5.1f} fps | w={row['w_fps']:5.1f} fps")
