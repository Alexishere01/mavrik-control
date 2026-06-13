import json

with open('Team1_VTOL.json', 'r') as f:
    data = json.load(f)

for name, comp in data.get('components', {}).items():
    loc = comp.get('location[ft]')
    if loc is not None:
        print(f"{name}: location = {loc}")
