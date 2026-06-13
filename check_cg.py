import json

with open("Team1_VTOL.json", "r") as f:
    data = json.load(f)

sum_w = 0
sum_wx = 0
sum_wy = 0
sum_wz = 0

for name, comp in data["components"].items():
    if "weight[lbf]" in comp:
        w = comp["weight[lbf]"]
        loc = comp["location[ft]"]
        sum_w += w
        sum_wx += w * loc[0]
        sum_wy += w * loc[1]
        sum_wz += w * loc[2]

print(f"Total Weight: {sum_w} lbf")
print(f"CG_x: {sum_wx/sum_w} ft")
print(f"CG_y: {sum_wy/sum_w} ft")
print(f"CG_z: {sum_wz/sum_w} ft")
