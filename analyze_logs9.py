import pandas as pd
df = pd.read_csv('datalog.csv')
df_armed = df[df['throttleFwdRight commanded'] > 0.05]
if len(df_armed) > 0:
    import numpy as np
    def quat_to_euler(e0, ex, ey, ez):
        theta = np.arcsin(np.clip(2 * (e0 * ey - ez * ex), -1.0, 1.0))
        return theta * 180/np.pi

    pitch = quat_to_euler(df_armed['e0'], df_armed['ex'], df_armed['ey'], df_armed['ez'])
    pitch_mean = pitch.mean()
    
    fr = df_armed['throttleFwdRight commanded'].mean()
    fl = df_armed['throttleFwdLeft commanded'].mean()
    ar = df_armed['throttleAftRight commanded'].mean()
    al = df_armed['throttleAftLeft commanded'].mean()
    
    print(f"Pitch: {pitch_mean:.1f}")
    print(f"Front: FR={fr:.3f}, FL={fl:.3f}")
    print(f"Rear:  AR={ar:.3f}, AL={al:.3f}")
    
    if fr > ar:
        print("Front thrust > Rear thrust -> Controller is pushing front UP (trying to pitch UP).")
    else:
        print("Rear thrust > Front thrust -> Controller is pushing rear UP (trying to pitch DOWN)!! MAPPING IS REVERSED!")
