import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

# --- Load recorded robot data ---
# Make sure to update the path to where your corner_visualization.csv is saved
df = pd.read_csv(
    r'../tests/pure_pursuit/data/corner_visualization.csv'
)

plt.figure(figsize=(8, 8))

# --- Ideal 90-Degree Corner Reference ---
segment_length = 40
# Segment 1: (0,0) to (40,0)
# Segment 2: (40,0) to (40,40)
path_x = [0, segment_length, segment_length]
path_y = [0, 0, segment_length]

plt.plot(path_x, path_y, 'r--', linewidth=2, label='Ideal 90° Corner Path')

# --- Actual robot trajectory ---
plt.plot(df['robot_x'], df['robot_y'], 'b-', linewidth=2, label='Robot Trajectory')

# --- Heading arrows every ~15 samples ---
# Using 15 to better capture the rapid turn at the corner
idx = range(0, len(df), 15)
plt.quiver(
    df['robot_x'].iloc[idx], df['robot_y'].iloc[idx],
    np.cos(df['theta'].iloc[idx]),
    np.sin(df['theta'].iloc[idx]),
    color='teal',
    scale=25,
    label='Robot Heading'
)

# --- Visualization of the "Corner Cutting" ---
# Optional: Plot the lookahead points to see what the robot was "seeing"
plt.scatter(df['lookahead_x'].iloc[idx], df['lookahead_y'].iloc[idx], 
            color='orange', s=10, alpha=0.5, label='Lookahead Points')

plt.axis('equal')
plt.legend()
plt.title("Pure Pursuit: 90° Corner Stress Test")
plt.xlabel("X (mm)")
plt.ylabel("Y (mm)")
plt.grid(True)
plt.tight_layout()
plt.show()