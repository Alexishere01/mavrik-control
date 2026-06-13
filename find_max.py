import pandas as pd
df = pd.read_csv('datalog.csv')
print(f"Max throttle: {df['throttleFwdRight commanded'].max()}")
print(f"Time range: {df['t'].min()} to {df['t'].max()}")
