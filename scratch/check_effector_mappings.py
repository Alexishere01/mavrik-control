import json

with open('Team1_VTOL.json', 'r') as f:
    data = json.load(f)

for name, comp in data.get('components', {}).items():
    comp_str = json.dumps(comp)
    for effector in ['throttleFwdRight', 'throttleFwdLeft', 'throttleAftRight', 'throttleAftLeft']:
        if effector in comp_str:
            print(f"Component '{name}' references '{effector}'")
