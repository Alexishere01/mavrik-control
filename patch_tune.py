import sys
content = open('auto_tune.py').read()

# Fix Q_M_THST_HOVER from 0.125 to 0.58
content = content.replace("'Q_M_THST_HOVER':   0.125,", "'Q_M_THST_HOVER':   0.58,")

# Add RC_CHANNELS_OVERRIDE for "roll out" maneuver
import_code = """
def mav_rc_override(roll, pitch, throttle, yaw):
    # Channel mapping: 1=Roll, 2=Pitch, 3=Throttle, 4=Yaw
    import struct
    target_sys, target_comp = 1, 1
    # Pad to exactly 8 channels (0 = ignore)
    pwm = [roll, pitch, throttle, yaw, 0, 0, 0, 0]
    payload = struct.pack('<HHHHHHHHBB', *pwm, target_sys, target_comp)
    msg_id  = 70    # RC_CHANNELS_OVERRIDE
    crc_extra = 124  # MAVLink-defined for msg_id 70
    _send_mav(_mav_pkt(msg_id, payload, crc_extra))
"""
if "def mav_rc_override" not in content:
    content = content.replace("def mav_arm", import_code.strip() + "\n\ndef mav_arm")

# Inject roll-out logic after arming
roll_out_logic = """
        print(f"  [mavrik] Hovering for 5s...")
        time.sleep(5)
        print(f"  [mavrik] Injecting right-roll maneuver (RC Roll=1650) for 2s...")
        mav_rc_override(1650, 1500, 1500, 1500)
        time.sleep(2)
        print(f"  [mavrik] Centering sticks (RC Roll=1500), recovering for {flight_secs - 7}s...")
        mav_rc_override(1500, 1500, 1500, 1500)
        time.sleep(flight_secs - 7)
"""
content = content.replace("""        print(f"  [mavrik] Running for {flight_secs}s post-arm...")
        time.sleep(flight_secs)""", roll_out_logic.strip())

# Update objective bounds to focus on heavy-lifter pendulum dynamics
bounds_old = """
        rat_p    = trial.suggest_float('rat_p',    0.005, 0.10,  log=True)
        rat_d    = trial.suggest_float('rat_d',    0.001, 0.04,  log=True)
        rat_i    = trial.suggest_float('rat_i',    0.001, 0.015, log=True)
        ang_p    = trial.suggest_float('ang_p',    1.5,   5.0)
"""
bounds_new = """
        # Pendulum-focused bounds: Low P, High D, Fast I
        rat_p    = trial.suggest_float('rat_p',    0.010, 0.060, log=True)
        rat_d    = trial.suggest_float('rat_d',    0.005, 0.025, log=True)
        rat_i    = trial.suggest_float('rat_i',    0.005, 0.025, log=True)
        ang_p    = trial.suggest_float('ang_p',    2.0,   3.5)
        fltd     = trial.suggest_float('fltd',     3.0,   8.0)
"""
content = content.replace(bounds_old.strip(), bounds_new.strip())

overrides_add = """
            'Q_A_RAT_PIT_FLTD': fltd,
            'Q_A_RAT_RLL_FLTD': fltd,
"""
if "'Q_A_RAT_PIT_FLTD'" not in content:
    content = content.replace("'Q_A_RAT_RLL_IMAX': min(rat_p * 20, 0.20),", "'Q_A_RAT_RLL_IMAX': min(rat_p * 20, 0.20)," + overrides_add)

open('auto_tune.py', 'w').write(content)
print("Patched auto_tune.py")
