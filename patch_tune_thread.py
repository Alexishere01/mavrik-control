import re

content = open('auto_tune.py').read()

# Remove the previously added single mav_rc_override function
content = re.sub(r'def mav_rc_override.*?\n\n(?=def mav_arm)', '', content, flags=re.DOTALL)

# Inject the background thread and lock variables near imports
thread_code = """
import threading

_rc_override_lock = threading.Lock()
_rc_override_channels = [1500, 1500, 1500, 1500, 0, 0, 0, 0]
_rc_override_stop = threading.Event()
_rc_thread = None

def rc_override_thread_fn():
    global _rc_override_channels, _rc_override_stop
    import socket, struct
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while not _rc_override_stop.is_set():
        with _rc_override_lock:
            ch = list(_rc_override_channels)
        target_sys, target_comp = 1, 1
        payload = struct.pack('<HHHHHHHHBB', *ch, target_sys, target_comp)
        pkt = _mav_pkt(70, payload, 124)
        try:
            s.sendto(pkt, ('127.0.0.1', 14552))
        except Exception:
            pass
        time.sleep(0.02)
    s.close()
"""

# Let's insert thread_code right above load_parm_template
content = content.replace("def load_parm_template():", thread_code.strip() + "\n\ndef load_parm_template():")

# Start the thread in start_stack()
start_stack_inject = """
    print(f"  [stack] Starting RC override thread...")
    global _rc_override_stop, _rc_thread, _rc_override_channels
    with _rc_override_lock:
        _rc_override_channels = [1500, 1500, 1500, 1500, 0, 0, 0, 0]
    _rc_override_stop.clear()
    _rc_thread = threading.Thread(target=rc_override_thread_fn, daemon=True)
    _rc_thread.start()

    print(f"  [stack] Waiting {STACK_READY_SECS}s for all services to be ready...")
"""
content = content.replace('    print(f"  [stack] Waiting {STACK_READY_SECS}s for all services to be ready...")', start_stack_inject.strip())

# Stop the thread in kill_all()
kill_all_inject = """
    global _rc_override_stop, _rc_thread
    if _rc_thread is not None:
        _rc_override_stop.set()
        _rc_thread.join(timeout=1.0)
        _rc_thread = None

    for p in _procs:
"""
content = content.replace("    for p in _procs:", kill_all_inject.strip())

# Inject the roll-out logic using the thread channels
roll_out_logic_thread = """
        print(f"  [mavrik] Hovering for 5s...")
        time.sleep(5)
        print(f"  [mavrik] Injecting right-roll maneuver (RC Roll=1650) for 2s...")
        with _rc_override_lock:
            _rc_override_channels[0] = 1650
        time.sleep(2)
        print(f"  [mavrik] Centering sticks (RC Roll=1500), recovering for {flight_secs - 7}s...")
        with _rc_override_lock:
            _rc_override_channels[0] = 1500
        time.sleep(flight_secs - 7)
"""
# Replace the previous roll-out logic
content = re.sub(r'print\(f"  \[mavrik\] Hovering for 5s\.\.\."\).*?time\.sleep\(flight_secs - 7\)', roll_out_logic_thread.strip(), content, flags=re.DOTALL)

open('auto_tune.py', 'w').write(content)
print("Patched auto_tune.py with background RC thread.")
