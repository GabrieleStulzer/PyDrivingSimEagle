import os
import pandas as pd

# Cartella con i file di training
training_dir = 'logs/training_only_lateral/'

# Ottieni i file ordinati
files = sorted([f for f in os.listdir(training_dir) if f.endswith('_training.csv')])

# Dividi i file in indici dispari (1,3,5,...) e pari (2,4,6,...)
odd_files = [files[i] for i in range(0, len(files), 2)]      # indici 0, 2, 4, ... (file 1, 3, 5, ...)
even_files = [files[i] for i in range(1, len(files), 2)]     # indici 1, 3, 5, ... (file 2, 4, 6, ...)

print(f"File dispari (1, 3, 5, ...): {len(odd_files)}")
for f in odd_files:
    print(f"  - {f}")

print(f"\nFile pari (2, 4, 6, ...): {len(even_files)}")
for f in even_files:
    print(f"  - {f}")

# Unisci i file dispari
print("\nUnendo file dispari...")
dfs_odd = []
for f in odd_files:
    df = pd.read_csv(os.path.join(training_dir, f))
    dfs_odd.append(df)
    
merged_odd = pd.concat(dfs_odd, ignore_index=True)
output_odd = os.path.join(training_dir, 'training_odd.csv')
merged_odd.to_csv(output_odd, index=False)
print(f"✓ Salvato: {output_odd} ({len(merged_odd)} righe)")

# Unisci i file pari
print("\nUnendo file pari...")
dfs_even = []
for f in even_files:
    df = pd.read_csv(os.path.join(training_dir, f))
    dfs_even.append(df)
    
merged_even = pd.concat(dfs_even, ignore_index=True)
output_even = os.path.join(training_dir, 'training_even.csv')
merged_even.to_csv(output_even, index=False)
print(f"✓ Salvato: {output_even} ({len(merged_even)} righe)")

print("\nMerge completato!")
