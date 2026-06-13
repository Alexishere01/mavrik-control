import pandas as pd
df = pd.read_csv('bridge_diag.csv')
armed_rows = df[df['armed'] == 1]
if len(armed_rows) > 0:
    first_armed_idx = armed_rows.index[0]
    flight_df = df.iloc[first_armed_idx :: 40]
    print("Tilt servos (downsampled):")
    for idx, row in flight_df.iterrows():
        print(f"sim_t={row['sim_t']:5.2f}s | yaw={row['yaw_deg']:6.2f}° | CH8(L)={row['ap_pwm8']:4.0f} | CH9(R)={row['ap_pwm9']:4.0f} | flapsL={row['flapsL']:.3f} | flapsR={row['flapsR']:.3f}")
