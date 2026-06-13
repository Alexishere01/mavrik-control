import pandas as pd
import numpy as np

def quat_to_euler(e0, ex, ey, ez):
    phi = np.arctan2(2 * (e0 * ex + ey * ez), 1 - 2 * (ex**2 + ey**2))
    theta = np.arcsin(np.clip(2 * (e0 * ey - ez * ex), -1.0, 1.0))
    psi = np.arctan2(2 * (e0 * ez + ex * ey), 1 - 2 * (ey**2 + ez**2))
    return phi * 180/np.pi, theta * 180/np.pi, psi * 180/np.pi

try:
    df = pd.read_csv('bridge_diag.csv')
    df_armed = df[df['ap_has_armed'] == True]
    if len(df_armed) > 0:
        first_armed_t = df_armed['sim_t'].min()
        print(f"AP Armed at sim_t = {first_armed_t:.2f}s")
        
        # Look at the transition window in datalog.csv (around 10-18 seconds!)
        df_data = pd.read_csv('datalog.csv')
        df_data['roll'], df_data['pitch'], df_data['yaw'] = quat_to_euler(
            df_data['e0'], df_data['ex'], df_data['ey'], df_data['ez']
        )
        
        # Let's find when handover actually happened by looking for when ArduPilot starts commanding motors in bridge_diag.csv
        # In bridge_diag.csv, look for when 'has_armed' (which is the bridge's handover state) becomes 1.
        df_handover = df[df['armed'] == 1]
        if len(df_handover) > 0:
            handover_t = df_handover['sim_t'].min()
            print(f"Bridge Handover Triggered at sim_t = {handover_t:.2f}s")
            
            # Print datalog during transition with labels
            df_transition = df_data[(df_data['t'] >= 31.0) & (df_data['t'] <= 38.0)].copy()
            for _, r in df_transition.iterrows():
                print(f"t={r['t']:5.2f}s | roll={r['roll']:7.2f}° | pitch={r['pitch']:7.2f}° | yaw={r['yaw']:7.2f}° | ail_cmd={r['aileron commanded']:6.2f}° | ele_cmd={r['elevator commanded']:6.2f}° | rud_cmd={r['rudder commanded']:6.2f}°")
        else:
            print("No handover detected in bridge_diag.csv")
    else:
        print("No armed data in bridge_diag.csv")
except Exception as e:
    print(e)
