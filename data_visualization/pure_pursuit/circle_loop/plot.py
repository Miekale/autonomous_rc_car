"""
Import from tests/pure_pursuit/testLoop circle tracking
"""

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

df = pd.read_csv(r'C:\Users\mckal\OneDrive\Documents\Mechatronics\3B\autonomous_rc_car\tests\pure_pursuit\data\circle_tracking_data.csv')
plt.figure(figsize=(8,8))

# target circle
theta = np.linspace(0, 2*np.pi, 200)
plt.plot(20*np.cos(theta), 20*np.sin(theta), 'r--', label='Ideal Path (R=20)')

# actual trajectory
plt.plot(df['x'], df['y'], 'b-', label='Robot Trajectory')

# just orientation
idx = range(0, len(df), 20)
plt.quiver(df['x'][idx], df['y'][idx], 
           np.cos(df['theta'][idx]), np.sin(df['theta'][idx]), 
           color='blue', scale=20, label='Robot Heading')

plt.axis('equal')
plt.legend()
plt.title("Pure Pursuit: Circular Path Tracking")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.grid(True)
plt.show()