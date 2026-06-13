import pandas as pd
df = pd.read_csv('datalog.csv')
print("Columns:", df.columns.tolist())
print(f"Total rows: {len(df)}")
print("Max alt:", df['position_z'].min() * -1) # Z is NED, so -Z is alt
