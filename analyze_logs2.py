import pandas as pd
df = pd.read_csv('bridge_diag.csv')
print(df.columns.tolist())
print(f"Total rows: {len(df)}")
if 'ap_has_armed' in df.columns:
    print(f"Armed rows: {len(df[df['ap_has_armed'] == True])}")
if 'motor_active' in df.columns:
    print(f"Motor active rows: {len(df[df['motor_active'] == True])}")
