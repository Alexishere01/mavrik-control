import pandas as pd
import glob
import os

all_files = glob.glob("tune_results*.csv")
best_tunes = pd.DataFrame()

for file in all_files:
    try:
        df = pd.read_csv(file)
        if 'score' in df.columns:
            high_scores = df[df['score'] >= 100]
            if not high_scores.empty:
                high_scores['source_file'] = file
                best_tunes = pd.concat([best_tunes, high_scores])
    except Exception as e:
        print(f"Error reading {file}: {e}")

if not best_tunes.empty:
    best_tunes = best_tunes.sort_values(by='score', ascending=False)
    print("Found best tunes:")
    print(best_tunes[['source_file', 'trial', 'score', 'rat_p', 'rat_d', 'rat_i', 'ang_p']].head(10))
    os.makedirs('test_results', exist_ok=True)
    best_tunes.to_csv('test_results/best_tunes.csv', index=False)
    print("\nSaved to test_results/best_tunes.csv")
else:
    print("No tunes found with score >= 100")
