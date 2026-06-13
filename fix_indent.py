with open('auto_tune.py', 'r') as f:
    lines = f.readlines()
for i, line in enumerate(lines):
    if line.startswith('print(f"  [mavrik] Hovering for 5s...")'):
        lines[i] = '        ' + line
with open('auto_tune.py', 'w') as f:
    f.writelines(lines)
