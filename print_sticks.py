import pandas as pd
df = pd.read_csv('bridge_diag.csv')
armed_rows = df[df['armed'] == 1]
if len(armed_rows) > 0:
    first_armed_idx = armed_rows.index[0]
    flight_df = df.iloc[first_armed_idx :: 40]
    print("Stick inputs:")
    for idx, row in flight_df.iterrows():
        print(f"sim_t={row['sim_t']:5.2f}s | pitch={row['pitch_deg']:6.2f}° | ele(stick)={row['ele']:6.2f} | ail(stick)={row['ail']:6.2f} | rud(stick)={row['rud']:6.2f}")
