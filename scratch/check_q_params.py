with open('mavrik.parm', 'r') as f:
    for line in f:
        line = line.strip()
        if line and not line.startswith('#'):
            parts = line.split()
            if len(parts) >= 2:
                name, val = parts[0], parts[1]
                if name.startswith('Q_'):
                    print(f"{name:<20} {val}")
