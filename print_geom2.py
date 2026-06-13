import json
with open('Team1_VTOL.json', 'r') as f:
    geom = json.load(f)
print(list(geom.keys()))
if 'vehicle' in geom:
    print(list(geom['vehicle'].keys()))
