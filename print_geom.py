import json
with open('Team1_VTOL.json', 'r') as f:
    geom = json.load(f)

print("Mass properties:")
print(geom.get('mass_properties', {}))

print("\nMotor VTOL positions:")
for name, m in geom.get('motors', {}).items():
    if 'vtol' in name.lower() or 'tilt' in name.lower():
        print(f"  {name}: {m.get('position[ft]')}")

print("\nControl effectors magnitude limits:")
for name, e in geom.get('control_effectors', {}).items():
    print(f"  {name}: {e.get('magnitude_limits', e.get('magnitude_limits[deg]'))}")
