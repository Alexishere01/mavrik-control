import sys
content = open('auto_tune.py').read()

# Replace the roll-out logic block with a robust polling loop
old_logic = """        print(f"  [mavrik] Hovering for 5s...")
        time.sleep(5)
        print(f"  [mavrik] Injecting right-roll maneuver (RC Roll=1650) for 2s...")
        mav_rc_override(1650, 1500, 1500, 1500)
        time.sleep(2)
        print(f"  [mavrik] Centering sticks (RC Roll=1500), recovering for {flight_secs - 7}s...")
        mav_rc_override(1500, 1500, 1500, 1500)
        time.sleep(flight_secs - 7)"""

new_logic = """
        print(f"  [mavrik] Sending RC overrides and holding hover for 5s...")
        start_t = time.time()
        while time.time() - start_t < 5.0:
            mav_rc_override(1500, 1500, 1500, 1500)
            time.sleep(0.1)
            
        print(f"  [mavrik] Injecting right-roll maneuver (RC Roll=1650) for 2s...")
        start_t = time.time()
        while time.time() - start_t < 2.0:
            mav_rc_override(1650, 1500, 1500, 1500)
            time.sleep(0.1)
            
        print(f"  [mavrik] Centering sticks (RC Roll=1500), recovering for {flight_secs - 7}s...")
        start_t = time.time()
        while time.time() - start_t < (flight_secs - 7):
            mav_rc_override(1500, 1500, 1500, 1500)
            time.sleep(0.1)
"""

if "Hovering for 5s..." in content:
    content = content.replace(old_logic, new_logic.strip())
    
open('auto_tune.py', 'w').write(content)
print("Patched auto_tune.py again!")
