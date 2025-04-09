import matplotlib.pyplot as plt
import csv

# Read the CSV file to get the data
time_steps = []
errors = []
v_cam_data = {'vx': [], 'vy': [], 'vz': [], 'wx': [], 'wy': [], 'wz': []}  # To store camera velocities vx, vy, vz, wx, wy, wz

# Read data from CSV file
with open("simulation_data.csv", mode='r') as f:
    reader = csv.reader(f)
    header = next(reader)  # Skip the header row
    for row in reader:
        time_steps.append(float(row[0]))  # Time step
        errors.append(float(row[1]))  # Error value
        for i in range(6):
            # Append to the specific velocity type based on the column index
            v_cam_data[list(v_cam_data.keys())[i]].append(float(row[i+2]))  # Camera velocities vx, vy, ..., wz

# Create the figure and axes for the two plots
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

# Plot 1: Error vs Time
ax1.plot(time_steps, errors, label='Pixel Error', color='blue')
ax1.set_title("Error vs Time")
ax1.set_xlabel("Time Steps")
ax1.set_ylabel("Error Magnitude")
ax1.grid(True)
ax1.legend()

# Plot 2: Camera Velocity Commands vs Time
for key in v_cam_data:
    ax2.plot(time_steps, v_cam_data[key], label=key)
ax2.set_title("Camera Velocity Commands vs Time")
ax2.set_xlabel("Time Steps")
ax2.set_ylabel("Velocity")
ax2.grid(True)
ax2.legend()

# Adjust layout for better spacing
plt.tight_layout()

# Show the plots
plt.show()
