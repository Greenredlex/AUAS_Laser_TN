import pandas as pd

a = pd.read_csv('Corrosie_meetingen\\Scan_data_corrosieC_meting5.csv')
print(a['z_values'][0])
z_list = [float(x) for x in a['z_values'][0].strip('[]').split(', ')]
print(len(z_list))

