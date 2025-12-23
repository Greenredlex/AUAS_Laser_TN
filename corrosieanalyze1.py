import sys
import csv
import json
import numpy as np
import matplotlib.pyplot as plt
import glob
import os
from scipy.signal import detrend
from scipy.interpolate import interp1d
from concurrent.futures import ThreadPoolExecutor, as_completed
from functools import lru_cache

# --- CONFIGURATIE ---
Z_THRESHOLD = -2.5       # Alles lager dan dit is tafel/achtergrond
POINTS_THRESHOLD = 500   # Hoeveel punten moeten > Z_THRESHOLD zijn om te zeggen: "Hier begint het blokje"
INTERP_STEP = 0.2        # We herschalen alle data naar 1 punt per 0.2mm voor het middelen
MAX_LENGTH = 160         # Hoe lang is een blokje max in mm? (voor de x-as limiet van de plot)

# Lijst met targets: 1 t/m 7 en C
TARGETS = [str(i) for i in range(1, 8)] + ['C'] 

def calculate_ra_roughness(x_vals, z_vals, robot_z):
    """ Berekent Ra van geldig oppervlak (Z > Threshold) """
    actual_z = z_vals + robot_z
    mask = actual_z > Z_THRESHOLD
    
    z_roi = z_vals[mask]
    if len(z_roi) < POINTS_THRESHOLD / 4: # Beetje coulance, maar moet wel data zijn
        return None, False # False betekent: dit is nog geen blokje

    # Detrending (kanteling weghalen)
    try:
        z_detrended = detrend(z_roi, type='linear')
    except:
        return None, False

    ra = np.mean(np.abs(z_detrended))
    
    # Return True als we denken dat we OP het blokje zitten (veel punten)
    is_on_block = len(z_roi) > POINTS_THRESHOLD
    return ra, is_on_block

def get_aligned_data(filepath):
    """
    Leest een file, berekent Ra per lijn, en lijnt de Y-as uit
    zodat Y=0 het begin van het blokje is.
    """
    try:
        with open(filepath, 'r') as f:
            reader = csv.DictReader(f)
            frames = list(reader)
    except:
        return None, None

    if not frames or 'robot_y' not in frames[0]:
        return None, None

    # Eerst data verzamelen - optimized parsing
    parsed_data = []
    
    for row in frames:
        try:
            xv = np.array(json.loads(row['x_values']), dtype=np.float32)
            zv = np.array(json.loads(row['z_values']), dtype=np.float32)
            ry = float(row['robot_y'])
            rz = float(row['robot_z'])
            parsed_data.append((ry, xv, zv, rz))
        except:
            continue

    if not parsed_data:
        return None, None

    raw_y = []
    raw_ra = []
    start_y = None

    # Loop door data om startpunt te vinden en Ra te berekenen
    for ry, xv, zv, rz in parsed_data:
        ra, is_on_block = calculate_ra_roughness(xv, zv, rz)
        
        if ra is not None:
            # Detecteer start van blokje (Edge Detection)
            if start_y is None and is_on_block:
                start_y = ry
            
            # Alleen data opslaan als we het startpunt hebben gevonden (of net gevonden hebben)
            if start_y is not None:
                # Zorg dat we positief blijven tellen (absolute afstand vanaf start)
                # Dit werkt voor scanners die vooruit of achteruit bewegen
                rel_y = abs(ry - start_y)
                
                if rel_y < MAX_LENGTH: # Niet oneindig doorgaan
                    raw_y.append(rel_y)
                    raw_ra.append(ra)

    if raw_y:
        return np.array(raw_y, dtype=np.float32), np.array(raw_ra, dtype=np.float32)
    return None, None

def process_file_to_interpolated(fpath, common_y_grid):
    """
    Process a single file and return interpolated data on common grid.
    Returns None if processing fails.
    """
    y_data, ra_data = get_aligned_data(fpath)
    
    if y_data is None or len(y_data) <= 10:
        return None
    
    # Sorteer op Y voor interpolatie (cruciaal)
    sort_idx = np.argsort(y_data)
    y_sorted = y_data[sort_idx]
    ra_sorted = ra_data[sort_idx]
    
    # Verwijder dubbele Y waarden voor interpolatie
    y_unique, unique_indices = np.unique(y_sorted, return_index=True)
    ra_unique = ra_sorted[unique_indices]

    # Interpoleer naar het gemeenschappelijke grid
    try:
        f_interp = interp1d(y_unique, ra_unique, kind='linear', bounds_error=False, fill_value=np.nan)
        ra_interp = f_interp(common_y_grid)
        return ra_interp
    except Exception as e:
        return None

def main():
    plt.figure(figsize=(14, 8))
    base_dirs = ['.', 'Corrosie_meetingen']
    colors = plt.cm.tab10(np.linspace(0, 1, len(TARGETS)))

    # Voor de Grand Average (totaal gemiddelde)
    all_blocks_interpolated = []
    common_y_grid = np.arange(0, MAX_LENGTH, INTERP_STEP, dtype=np.float32)

    # Gebruik ThreadPoolExecutor voor parallelle verwerking
    max_workers = min(32, (os.cpu_count() or 1) * 4)  # 4x CPU cores voor I/O bound werk
    
    for idx, target in enumerate(TARGETS):
        print(f"Verwerken Target {target}...")
        
        # Verzamel alle metingen voor dit blokje (meting1, meting2, etc)
        # Zoekpatronen (voor variaties in bestandsnaam)
        patterns = [
            f"*corrosie{target}_meting*.csv",
            f"*corrosie{target}_*.csv"
        ]
        
        files = []
        for p in patterns:
            for d in base_dirs:
                found = glob.glob(os.path.join(d, p))
                # Filter dubbelen en onzin
                for f in found:
                    if f not in files:
                        files.append(f)
        
        if not files:
            print(f" -> Geen files voor {target}")
            continue

        # Lijst om ge-interpoleerde curves op te slaan voor DIT blokje
        block_curves = []

        # Parallel processing van alle files
        with ThreadPoolExecutor(max_workers=max_workers) as executor:
            # Submit alle files voor verwerking
            future_to_file = {
                executor.submit(process_file_to_interpolated, fpath, common_y_grid): fpath 
                for fpath in files
            }
            
            # Verzamel resultaten zodra ze klaar zijn
            for future in as_completed(future_to_file):
                fpath = future_to_file[future]
                try:
                    ra_interp = future.result()
                    if ra_interp is not None:
                        block_curves.append(ra_interp)
                except Exception as e:
                    print(f" -> Error processing {fpath}: {e}")

        # Bereken gemiddelde voor dit blokje
        if block_curves:
            # Stapel ze: (Aantal_files, Aantal_punten)
            stack = np.vstack(block_curves)
            # NanMean negeert punten waar de ene file wel data had en de andere niet
            avg_ra_block = np.nanmean(stack, axis=0)
            
            # Plot de lijn voor dit blokje
            # Filter NaNs eruit voor de plot zodat de lijn niet breekt
            valid_mask = ~np.isnan(avg_ra_block)
            plt.plot(common_y_grid[valid_mask], avg_ra_block[valid_mask], 
                     label=f'Blok {target} (gem. van {len(block_curves)} scans)', 
                     color=colors[idx], alpha=0.7, linewidth=1.5)
            
            # Voeg toe aan Grand Total
            all_blocks_interpolated.append(avg_ra_block)
        else:
            print(f" -> Wel files, maar geen geldige uitgelijnde data voor {target}")

    # --- PLOT GRAND AVERAGE ---
    if all_blocks_interpolated:
        grand_stack = np.vstack(all_blocks_interpolated)
        grand_avg = np.nanmean(grand_stack, axis=0)
        
        valid_mask = ~np.isnan(grand_avg)
        plt.plot(common_y_grid[valid_mask], grand_avg[valid_mask], 
                 label='Totaal Gemiddelde (Alle Blokken)', 
                 color='black', linestyle='--', linewidth=3, zorder=10)

    plt.title(f"Gealineerde & Gemiddelde Ruwheid ($R_a$) - Uitgelijnd op start blokje", fontsize=20)
    plt.xlabel("Afstand vanaf begin blokje (mm)", fontsize=16)
    plt.ylabel("Gemiddelde Ruwheid $R_a$ (mm)", fontsize=16)
    plt.legend(bbox_to_anchor=(1.02, 1), loc='upper left', fontsize=14)
    plt.grid(True, which='major', linestyle='-', alpha=0.8)
    plt.grid(True, which='minor', linestyle=':', alpha=0.4)
    plt.minorticks_on()
    plt.tight_layout()
    plt.savefig('corrosie_analyze_plot.png', dpi=300, bbox_inches='tight')
    plt.show()

if __name__ == "__main__":
    main()