import pandas as pd
df = pd.read_csv('datalog.csv')
print(df[df['throttleFwdRight commanded'] > 0.5][['t', 'throttleFwdRight commanded']].head(10))
