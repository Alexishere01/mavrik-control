from pymavlink import mavutil

master = mavutil.mavlink_connection('mav.tlog')

print(f"{'Time':<10} | {'AP_Pitch':<10} | {'AP_PitchRate':<12} | {'PWM1-4':<25}")
print("-" * 65)

# We want to match ATTITUDE and SERVO_OUTPUT_RAW messages
last_pitch = None
last_pitchspeed = None
start_timestamp = None

while True:
    msg = master.recv_match()
    if msg is None:
        break
        
    msg_type = msg.get_type()
    
    if msg_type == 'ATTITUDE':
        if start_timestamp is None:
            # First message timestamp
            start_timestamp = getattr(msg, '_timestamp', 0)
        t = getattr(msg, '_timestamp', 0) - start_timestamp
        last_pitch = msg.pitch * 57.2957795
        last_pitchspeed = msg.pitchspeed * 57.2957795
        
    elif msg_type == 'SERVO_OUTPUT_RAW' and last_pitch is not None:
        t = getattr(msg, '_timestamp', 0) - start_timestamp
        pwms = [msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw]
        # Only print once we are close to the arming time (sim_t around 20s onwards)
        # We can just print all rows with non-zero PWM
        if any(p > 1000 for p in pwms):
            print(f"{t:10.2f} | {last_pitch:10.2f} | {last_pitchspeed:12.4f} | {pwms}")
            last_pitch = None
