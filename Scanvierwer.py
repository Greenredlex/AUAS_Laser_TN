import pandas as pd
import matplotlib.pyplot as plt

# Lees bestand in
data = pd.read_csv("scan_points_20251008_130732.csv")

# Extract kolommen
x = data["x"]
y = data["y"]
z = data["z"]
intensity = data["intensity"]


y = y/(500/25)

# Maak figuur
fig, ax = plt.subplots(figsize=(8, 6))


# Plot punten, kleur op basis van intensiteit
p = ax.scatter(x, y, c=intensity)
fig.colorbar(p, ax=ax, label="Intensity")
plt.axis('equal')

ax.set_xlabel("X")
ax.set_ylabel("Y")
plt.title("2D Scatter Plot of Laser Data")
plt.show()