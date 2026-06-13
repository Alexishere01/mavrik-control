import pandas as pd
df = pd.read_csv('bridge_diag.csv')
if 'ap_has_armed' in df.columns:
    df_armed = df[df['ap_pwm1'] > 1100]
    if len(df_armed) > 0:
        # Check what the PWMs are doing when pitch is -12 deg
        pitch_mean = df_armed['pitch_deg'].mean()
        pwm1 = df_armed['ap_pwm1'].mean() # FR
        pwm3 = df_armed['ap_pwm3'].mean() # FL
        pwm2 = df_armed['ap_pwm2'].mean() # RL
        pwm4 = df_armed['ap_pwm4'].mean() # RR
        
        print(f"Pitch: {pitch_mean:.1f}")
        print(f"Front PWMs: FR={pwm1:.1f}, FL={pwm3:.1f}")
        print(f"Rear PWMs:  RL={pwm2:.1f}, RR={pwm4:.1f}")
        
        if pwm1 > pwm2:
            print("Front PWM > Rear PWM -> Ardupilot is trying to pitch UP.")
        else:
            print("Rear PWM > Front PWM -> Ardupilot is trying to pitch DOWN!! MAPPING IS REVERSED!")
