with open('auto_tune.py', 'r') as f:
    lines = f.readlines()
for i, line in enumerate(lines):
    if line.startswith('        start_t = time.time()'):
        pass
    elif line.startswith('start_t = time.time()') or line.startswith('while time.time()') or line.startswith('mav_rc_override(') or line.startswith('time.sleep(0.1)'):
        lines[i] = '        ' + line.lstrip()

with open('auto_tune.py', 'w') as f:
    f.writelines(lines)
