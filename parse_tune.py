import pandas as pd
df = pd.read_csv('tune_results.csv')
if len(df) == 0:
    print("No results found.")
else:
    # Filter for the most recent 10 trials
    recent = df.tail(10)
    best = recent.sort_values('score', ascending=False).head(5)
    print("Top 5 results from the last 10 trials:")
    for i, row in best.iterrows():
        print(f"Trial {row['trial']}: Score: {row['score']:.1f}")
        print(f"  P: {row['rat_p']:.4f}, I: {row['rat_i']:.4f}, D: {row['rat_d']:.5f}")
        print(f"  ANG_P: {row['ang_p']:.2f}")
        print(f"  Max Roll/Pitch Rate: {row['max_p']:.1f} / {row['max_q']:.1f} rad/s")
        print(f"  Time to crash: {row['time_to_crash']:.1f}s")
        print()
