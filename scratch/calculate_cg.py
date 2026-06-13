import json

with open('Team1_VTOL.json', 'r') as f:
    data = json.load(f)

total_mass = 0.0
weighted_x = 0.0
weighted_y = 0.0
weighted_z = 0.0

for name, comp in data.get('components', {}).items():
    mass = comp.get('weight[lbf]', comp.get('mass[lbm]', 0.0))
    loc = comp.get('location[ft]')
    if mass > 0.0 and loc is not None:
        total_mass += mass
        weighted_x += mass * loc[0]
        weighted_y += mass * loc[1]
        weighted_z += mass * loc[2]
        print(f"Component '{name}': weight={mass:.3f} lbf, loc={loc}")

if total_mass > 0.0:
    cg_x = weighted_x / total_mass
    cg_y = weighted_y / total_mass
    cg_z = weighted_z / total_mass
    print("\n" + "="*50)
    print(f"Calculated CG: [{cg_x:.6f}, {cg_y:.6f}, {cg_z:.6f}] ft")
    print(f"Total weight:  {total_mass:.3f} lbf")
    print("="*50)
else:
    print("No components with weight and location found!")
