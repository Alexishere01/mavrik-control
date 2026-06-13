import json
with open('Team1_VTOL.json', 'r') as f:
    geom = json.load(f)

for name, comp in geom.get('components', {}).items():
    if 'motor' in name.lower():
        print(f"{name}:")
        print(f"  location[ft]: {comp.get('location[ft]')}")
