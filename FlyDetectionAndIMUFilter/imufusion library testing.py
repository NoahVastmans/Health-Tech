import pandas as pd
import imufusion
import numpy as np

# --- 1. Load Data and Initialize ---
try:
    df = pd.read_csv('your_imu_data.csv')
except FileNotFoundError:
    print("Error: 'your_imu_data.csv' not found. Please create a CSV file with IMU data.")
    exit()

# Extract data into numpy arrays
accelerometer = df[['accel_x', 'accel_y', 'accel_z']].values
gyroscope = df[['gyro_x', 'gyro_y', 'gyro_z']].values
timestamps = df['timestamp'].values

# Calculate the time step
delta_time = np.mean(np.diff(timestamps))

# Initialize the AHRS filter
ahrs = imufusion.Ahrs()

# State variables
is_on_ground = True
is_launched = False
is_in_free_fall = False
vz = 0.0

# Thresholds (tune these)
LAUNCH_ACCEL_THRESHOLD = 20.0
FREEFALL_ACCEL_THRESHOLD = 1.5
LANDING_ACCEL_THRESHOLD = 15.0

print("Starting algorithm test with recorded data...")

# --- 2. Main Processing Loop ---
for i in range(len(accelerometer)):
    # Get current sensor data
    accel = accelerometer[i]
    gyro = gyroscope[i]

    # Update the AHRS filter with the new data
    ahrs.update_imu(gyro, accel, delta_time)

    # Get the gravity-removed linear acceleration
    linear_accel = ahrs.linear_acceleration
    vz_previous = vz
    vz += linear_accel[2] * delta_time

    # State machine logic
    if is_on_ground and np.linalg.norm(linear_accel) > LAUNCH_ACCEL_THRESHOLD:
        is_on_ground = False
        is_launched = True
        vz = 0.0 # Reset velocity
        print(f"[{timestamps[i]:.2f}s] LAUNCH DETECTED")

    elif is_launched and np.linalg.norm(linear_accel) < FREEFALL_ACCEL_THRESHOLD:
        is_launched = False
        is_in_free_fall = True
        print(f"[{timestamps[i]:.2f}s] FREE FALL / APEX PHASE")

    if is_in_free_fall:
        # Apex detection (check for zero-crossing of vertical velocity)
        if vz < 0.0 and vz_previous >= 0.0:
            print(f"[{timestamps[i]:.2f}s] APEX REACHED")
            print(f"  - Orientation: Roll={ahrs.roll:.2f}°, Pitch={ahrs.pitch:.2f}°, Yaw={ahrs.yaw:.2f}°")

    if is_in_free_fall and np.linalg.norm(linear_accel) > LANDING_ACCEL_THRESHOLD:
        is_in_free_fall = False
        is_on_ground = True
        print(f"[{timestamps[i]:.2f}s] LANDING DETECTED")
        print("--------------------------------------------------")