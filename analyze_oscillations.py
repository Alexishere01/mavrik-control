import pandas as pd
df = pd.read_csv('bridge_diag.csv')
armed_rows = df[df['armed'] == 1]
if len(armed_rows) > 0:
    first_armed_idx = armed_rows.index[0]
    t_handover = df.iloc[first_armed_idx]['sim_t']
    print(f"Handover occurred at sim_t = {t_handover:.2f}s")
    
    # Print telemetry from 1.0s after handover up to 10s after handover
    post_blend_df = df[df['sim_t'] > t_handover + 1.0]
    if len(post_blend_df) > 0:
        print("\n Telemetry during/after blend transition (first 100 rows):")
        print(post_blend_df[['sim_t', 'roll_deg', 'pitch_deg', 'yaw_deg', 'ap_pwm1', 'ap_pwm2', 'ap_pwm3', 'ap_pwm4', 'thrFR', 'thrFL']].head(100).to_string())
    else:
        print("No post-blend telemetry found!")
else:
    print("No handover found!")
