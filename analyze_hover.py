import csv

try:
    with open('bridge_diag.csv', 'r') as f:
        reader = csv.reader(f)
        header = next(reader)
        header = [h.strip() for h in header]
        
        c7_idx = header.index('c[7]')
        c8_idx = header.index('c[8]')
        armed_idx = header.index('armed')
        roll_idx = header.index('roll')
        pitch_idx = header.index('pitch')
        
        rows = list(reader)
        
        c7_early = [float(row[c7_idx]) for row in rows[:50]]
        c8_early = [float(row[c8_idx]) for row in rows[:50]]
        print(f"c[7] early range: {min(c7_early)} to {max(c7_early)}")
        print(f"c[8] early range: {min(c8_early)} to {max(c8_early)}")
        
        armed_transition = -1
        for i in range(1, len(rows)):
            if int(rows[i-1][armed_idx]) == 0 and int(rows[i][armed_idx]) == 1:
                armed_transition = i
                break
                
        if armed_transition != -1:
            roll_at_arm = float(rows[armed_transition][roll_idx])
            pitch_at_arm = float(rows[armed_transition][pitch_idx])
            print(f"Roll at arm: {roll_at_arm:.2f}°")
            print(f"Pitch at arm: {pitch_at_arm:.2f}°")
        else:
            print("Never armed!")
            
        armed_rows = [row for row in rows if int(row[armed_idx]) == 1]
        if len(armed_rows) > 0:
            stable_count = sum(1 for row in armed_rows if abs(float(row[roll_idx])) < 5 and abs(float(row[pitch_idx])) < 5)
            percentage = stable_count / len(armed_rows) * 100
            print(f"Hover quality: {percentage:.2f}% ({stable_count} / {len(armed_rows)})")
        else:
            print("No armed rows.")
            
except Exception as e:
    print(f"Error: {e}")
