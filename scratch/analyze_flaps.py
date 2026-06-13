import csv

with open('bridge_diag.csv', 'r') as f:
    reader = csv.DictReader(f)
    rows = list(reader)

arm_idx = -1
for i, r in enumerate(rows):
    if r['ap_has_armed'] == '1':
        arm_idx = i
        break

if arm_idx == -1:
    print("Never armed!")
else:
    print(f"{'sim_t':>7} {'pitch':>7} {'ap_pwm8':>7} {'ap_pwm9':>7} {'flapsR':>7} {'flapsL':>7}")
    for r in rows[arm_idx::10]: # print every 10th row (approx 0.2s)
        print(f"{float(r['sim_t']):7.2f} {float(r['pitch_deg']):7.2f} {r['ap_pwm8']:>7} {r['ap_pwm9']:>7} "
              f"{float(r['flapsR']):7.4f} {float(r['flapsL']):7.4f}")
