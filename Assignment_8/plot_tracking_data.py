import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyArrowPatch
import math

# Load the CSV data
file_path = '/home/kohigashi/kulyah/sro/Assignment_8/robot_tracking_data_20250504_194014.csv'
df = pd.read_csv(file_path)

# Convert timestamp to relative time in seconds
df['rel_time'] = df['timestamp'] - df['timestamp'].iloc[0]

# 1. Robot and target positions plot
plt.figure(figsize=(8, 6))
plt.title('Robot and Target Positions')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.grid(True)

# Plot robot path
plt.plot(df['robot_x'], df['robot_y'], 'b-', alpha=0.6, label='Robot Path')

# Plot target position
target_x, target_y = df['target_x'].iloc[0], df['target_y'].iloc[0]
plt.plot(target_x, target_y, 'rx', markersize=10, label='Target')

# Draw a circle to represent the target
target_circle = plt.Circle((target_x, target_y), 0.1, fill=False, color='r', linestyle='--')
plt.gca().add_patch(target_circle)

plt.axis('equal')
plt.legend()
plt.tight_layout()
plt.savefig('robot_position_plot.png', dpi=300)
plt.show()

# 2. Distance error plot
plt.figure(figsize=(8, 6))
plt.title('Distance Error over Time')
plt.xlabel('Time (s)')
plt.ylabel('Error Distance (m)')
plt.grid(True)
plt.plot(df['rel_time'], df['error_distance'], 'g-')
plt.tight_layout()
plt.savefig('distance_error_plot.png', dpi=300)
plt.show()

# 3. Position plot with orientation
plt.figure(figsize=(8, 6))
plt.title('Robot Path with Orientation')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.grid(True)

# Plot full robot path
plt.plot(df['robot_x'], df['robot_y'], 'b-', alpha=0.4, label='Robot Path')

# Plot target
plt.plot(target_x, target_y, 'rx', markersize=10, label='Target')

# Add arrows to show orientation at regular intervals
step = max(1, len(df) // 20)  # Show about 20 arrows along the path
for i in range(0, len(df), step):
    # Only plot if we have valid orientation data
    if not math.isnan(df['robot_yaw'].iloc[i]):
        # Create arrow for robot orientation
        dx = 0.1 * math.cos(df['robot_yaw'].iloc[i])
        dy = 0.1 * math.sin(df['robot_yaw'].iloc[i])
        plt.arrow(df['robot_x'].iloc[i], df['robot_y'].iloc[i], dx, dy,
                 head_width=0.05, head_length=0.05, fc='blue', ec='blue', alpha=0.6)

# Mark final position
last_idx = len(df) - 1
while last_idx > 0 and (math.isnan(df['robot_x'].iloc[last_idx]) if last_idx < len(df) else True):
    last_idx -= 1
    
plt.plot(df['robot_x'].iloc[last_idx], df['robot_y'].iloc[last_idx], 'bo', markersize=8, label='Final Position')

# Add target circle
target_circle2 = plt.Circle((target_x, target_y), 0.1, fill=False, color='r', linestyle='--')
plt.gca().add_patch(target_circle2)

plt.axis('equal')
plt.legend()
plt.tight_layout()
plt.savefig('robot_orientation_plot.png', dpi=300)
plt.show()