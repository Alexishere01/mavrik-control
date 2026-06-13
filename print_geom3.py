import json
with open('Team1_VTOL.json', 'r') as f:
    geom = json.load(f)

for name, comp in geom.get('components', {}).items():
    if 'motor' in name.lower() or 'cg' in name.lower() or 'mass' in name.lower():
        print(f"{name}:")
        if 'position[ft]' in comp:
            print(f"  position[ft]: {comp['position[ft]']}")
        if 'mass[lbm]' in comp:
            print(f"  mass[lbm]: {comp['mass[lbm]']}")
