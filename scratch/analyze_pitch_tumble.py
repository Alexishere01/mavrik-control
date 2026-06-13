import csv

with open('bridge_diag.csv', 'r') as f:
    reader = csv.DictReader(f)
    rows = list(reader)

# Find when ap_has_armed becomes 1
arm_idx = -1
for i, r in enumerate(rows):
    if r['ap_has_armed'] == '1':
        arm_idx = i
        break

if arm_idx == -1:
    print("Never armed in log!")
    # Let's print the last 10 rows anyway
    start = max(0, len(rows) - 20)
    print_rows = rows[start:]
else:
    start = max(0, arm_idx - 5)
    end = min(len(rows), arm_idx + 200)
    print_rows = rows[start:end]

print(f"{'sim_t':>7} {'pitch':>7} {'q_rads':>7} {'ap1':>5} {'ap2':>5} {'ap3':>5} {'ap4':>5} {'ele':>6} {'thrFR':>6} {'thrFL':>6} {'thrAR':>6} {'thrAL':>6} {'armed':>5} {'act':>5}")
for r in print_rows:
    print(f"{float(r['sim_t']):7.2f} {float(r['pitch_deg']):7.2f} {float(r['q_rads']):7.4f} "
          f"{r['ap_pwm1']:>5} {r['ap_pwm2']:>5} {r['ap_pwm3']:>5} {r['ap_pwm4']:>5} "
          f"{float(r['ele']):6.2f} {float(r['thrFR']):6.3f} {float(r['thrFL']):6.3f} {float(r['thrAR']):6.3f} {float(r['thrAL']):6.3f} "
          f"{r['armed']:>5} {r['motor_active']:>5}")
