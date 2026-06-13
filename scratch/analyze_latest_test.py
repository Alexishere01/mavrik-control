import pandas as pd
import numpy as np

# Load the bridge diagnostics CSV
df = pd.read_csv('bridge_diag.csv')

print(f"Total rows in bridge_diag.csv: {len(df)}")
if len(df) == 0:
    print("Empty csv!")
    exit(0)

# Check if armed
armed_rows = df[df['ap_has_armed'] == 1]
if len(armed_rows) == 0:
    armed_rows = df[df['armed'] == 1]

if len(armed_rows) > 0:
    first_armed_idx = armed_rows.index[0]
    t_handover = df.iloc[first_armed_idx]['sim_t']
    print(f"Handover detected at index {first_armed_idx}, sim_t = {t_handover:.4f}s")
    
    # Slice from just before handover to the end
    sub_df = df.iloc[max(0, first_armed_idx - 10):]
    
    print("\nTelemetry overview from handover to end (spaced samples):")
    # Take up to 40 evenly spaced samples to avoid cluttering the output
    indices = np.linspace(0, len(sub_df) - 1, min(40, len(sub_df)), dtype=int)
    print(sub_df.iloc[indices][['sim_t', 'alt_ft', 'roll_deg', 'pitch_deg', 'yaw_deg', 'u_fps', 'v_fps', 'w_fps', 'ap_pwm1', 'ap_pwm2', 'ap_pwm3', 'ap_pwm4', 'ap_pwm8', 'ap_pwm9', 'ail', 'ele', 'rud']].to_string())
    
    # Check max deviation / state at the end
    last_row = df.iloc[-1]
    print(f"\nFinal state (sim_t = {last_row['sim_t']:.2f}s):")
    print(f"Alt: {last_row['alt_ft']:.2f} ft")
    print(f"Roll: {last_row['roll_deg']:.2f} deg, Pitch: {last_row['pitch_deg']:.2f} deg, Yaw: {last_row['yaw_deg']:.2f} deg")
    print(f"Velocities (u, v, w): {last_row['u_fps']:.2f}, {last_row['v_fps']:.2f}, {last_row['w_fps']:.2f} fps")
    print(f"RC Overrides (ail, ele, rud): {last_row['ail']}, {last_row['ele']}, {last_row['rud']}")
else:
    print("No arming / handover detected in bridge_diag.csv.")
    print("Showing last 20 rows of file:")
    print(df.tail(20)[['sim_t', 'alt_ft', 'roll_deg', 'pitch_deg', 'yaw_deg', 'u_fps', 'v_fps', 'w_fps', 'ail', 'ele', 'rud']].to_string())
