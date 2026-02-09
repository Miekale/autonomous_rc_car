"""
Import from tests/pure_pursuit/testLoop figure-8 tracking
"""

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

# --- Load recorded robot data ---
df = pd.read_csv(
    r'../tests/pure_pursuit/data/figure8_visualization.csv'
)

plt.figure(figsize=(8, 8))

# --- Ideal figure-8 reference (lemniscate style) ---
R = 20
t = np.linspace(0, 2*np.pi, 400)
x_ref = R * np.sin(t)
y_ref = R * np.sin(t) * np.cos(t)

plt.plot(x_ref, y_ref, 'r--', label='Ideal Figure-8 Path')

# --- Actual robot trajectory ---
plt.plot(df['x'], df['y'], 'b-', label='Robot Trajectory')

# --- Heading arrows every ~20 samples ---
idx = range(0, len(df), 20)
plt.quiver(
    df['x'][idx], df['y'][idx],
    np.cos(df['theta'][idx]),
    np.sin(df['theta'][idx]),
    scale=20,
    label='Robot Heading'
)

plt.axis('equal')
plt.legend()
plt.title("Pure Pursuit: Figure-8 Path Tracking")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.grid(True)
plt.show()
