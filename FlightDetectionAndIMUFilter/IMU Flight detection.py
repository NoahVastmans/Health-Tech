import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
#import math
import imufusion
#import time

# Global constants
g = 9.81  # Acceleration due to gravity (m/s^2)

#Global variables
accel = []
gyro = []
dt = 1/54  # Time step based on IMU sample rate (54 Hz)
F1_Raw = []  # To store acceleration magnitudes for Take1
F1_filtered = []  # To store filtered acceleration magnitudes for Take1
F2_Raw = []  # To store acceleration magnitudes for Take2
F2_filtered = []  # To store filtered acceleration magnitudes for Take2
F3_Raw = []  # To store acceleration magnitudes for Take3
F3_filtered = []  # To store filtered acceleration magnitudes for Take3

# Setting the sheets to read
sheet_to_read1 = '3 throws Acc take1'
sheet_to_read2 = '3 throws Gyr take1'   # <-- fixed: gyro for take1

sheet_to_read3 = '3 throws Acc take2'
sheet_to_read4 = '3 throws Gyr take2'

sheet_to_read5 = '3 throws Acc take3'
sheet_to_read6 = '3 throws Gyr take3'

# IMU Raw data values to read
columns_to_readAcc = ['X [mg]', 'Y [mg]', 'Z [mg]']  # For Accelerometer data
columns_to_readGyr = ['X [dps]', 'Y [dps]', 'Z [dps]'] # For Gyroscope data
time_column = 'Time [sec]' # timestamp column

# Read specified columns from the chosen sheet
Acc1 = pd.read_excel('Data 0904.xlsx', sheet_name=sheet_to_read1, usecols=columns_to_readAcc)
Gyr1 = pd.read_excel('Data 0904.xlsx', sheet_name=sheet_to_read2, usecols=columns_to_readGyr)
Time1 = pd.read_excel('Data 0904.xlsx', sheet_name=sheet_to_read1, usecols=[time_column])

Acc2 = pd.read_excel('Data 0904.xlsx', sheet_name=sheet_to_read3, usecols=columns_to_readAcc)
Gyr2 = pd.read_excel('Data 0904.xlsx', sheet_name=sheet_to_read4, usecols=columns_to_readGyr)
Time2 = pd.read_excel('Data 0904.xlsx', sheet_name=sheet_to_read3, usecols=[time_column])

Acc3 = pd.read_excel('Data 0904.xlsx', sheet_name=sheet_to_read5, usecols=columns_to_readAcc)
Gyr3 = pd.read_excel('Data 0904.xlsx', sheet_name=sheet_to_read6, usecols=columns_to_readGyr)
Time3 = pd.read_excel('Data 0904.xlsx', sheet_name=sheet_to_read5, usecols=[time_column])

print(Gyr3.head())  # Show the first few rows. Test to see if data is read correctly

# --- Replace processing: correct units, per-take AHRS, separate vectors & magnitudes ---
import imufusion

mg2ms2 = 9.81 / 1000.0
deg2rad = np.pi / 180.0

def to_ndarrays(Acc_df, Gyr_df, Time_df):
    acc_cols = ['X [mg]', 'Y [mg]', 'Z [mg]']
    gyr_cols = ['X [dps]', 'Y [dps]', 'Z [dps]']

    # use .values to get numpy.ndarray (n,3); ensure float dtype
    acc = Acc_df[acc_cols].values.astype(float) * mg2ms2      # accel in m/s^2
    gyr = Gyr_df[gyr_cols].values.astype(float)     # gyro in rad/s (from dps)

    # timestamps (fallback to index*dt)
    try:
        times = Time_df[time_column].values.astype(float)
    except Exception:
        times = np.arange(acc.shape[0]) * dt

    n = min(acc.shape[0], gyr.shape[0], times.shape[0])
    return acc[:n, :], gyr[:n, :], times[:n]

# build arrays for each take (now using .values)
acc1, gyr1, t1 = to_ndarrays(Acc1, Gyr1, Time1)
acc2, gyr2, t2 = to_ndarrays(Acc2, Gyr2, Time2)
acc3, gyr3, t3 = to_ndarrays(Acc3, Gyr3, Time3)

def process_take_arrays(acc_arr, gyr_arr, times):
    # acc_arr and gyr_arr are numpy.ndarray shape (n,3)
    n = acc_arr.shape[0]
    ahrs_local = imufusion.Ahrs()
    raw_vecs = np.array(acc_arr, copy=True)   # (n,3)
    filt_vecs = np.zeros((n,3), dtype=float)
    raw_mags = np.linalg.norm(raw_vecs, axis=1)
    filt_mags = np.zeros(n, dtype=float)

    for i in range(n):
        # make sure each sample is a 1-D numpy array with 3 elements
        accel_i = np.asarray(acc_arr[i], dtype=float).reshape(3,)
        gyro_i  = np.asarray(gyr_arr[i], dtype=float).reshape(3,)

        # sanity check
        if accel_i.shape != (3,) or gyro_i.shape != (3,):
            raise ValueError(f"Sample {i} has wrong shape: accel {accel_i.shape}, gyro {gyro_i.shape}")

        # pass 1-D numpy arrays (length-3) to the filter
        ahrs_local.update_imu(gyro_i, accel_i, dt)

        lin_acc = np.asarray(ahrs_local.linear_acceleration, dtype=float).reshape(3,)
        filt_vecs[i, :] = lin_acc
        filt_mags[i] = np.linalg.norm(lin_acc)

    return {
        'time': times,
        'raw_vecs': raw_vecs,
        'filt_vecs': filt_vecs,
        'raw_mags': raw_mags,
        'filt_mags': filt_mags
    }

# process
take1 = process_take_arrays(acc1, gyr1, t1)
take2 = process_take_arrays(acc2, gyr2, t2)
take3 = process_take_arrays(acc3, gyr3, t3)

# --- Plot magnitudes (raw dashed vs filtered solid) ---
fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)

# Take 1
axs[0].plot(take1['time'], take1['raw_mags'], linestyle='--', color='tab:blue', alpha=0.7, label='Take1 Raw')
axs[0].plot(take1['time'], take1['filt_mags'], linestyle='-', color='tab:blue', label='Take1 Filtered')
axs[0].set_title('Take 1 - Accelerometer: raw vs filtered magnitude')
axs[0].set_ylabel('Acc (m/s^2)')
axs[0].legend()
axs[0].grid(True)

# Take 2
axs[1].plot(take2['time'], take2['raw_mags'], linestyle='--', color='tab:orange', alpha=0.7, label='Take2 Raw')
axs[1].plot(take2['time'], take2['filt_mags'], linestyle='-', color='tab:orange', label='Take2 Filtered')
axs[1].set_title('Take 2 - Accelerometer: raw vs filtered magnitude')
axs[1].set_ylabel('Acc (m/s^2)')
axs[1].legend()
axs[1].grid(True)

# Take 3
axs[2].plot(take3['time'], take3['raw_mags'], linestyle='--', color='tab:green', alpha=0.7, label='Take3 Raw')
axs[2].plot(take3['time'], take3['filt_mags'], linestyle='-', color='tab:green', label='Take3 Filtered')
axs[2].set_title('Take 3 - Accelerometer: raw vs filtered magnitude')
axs[2].set_xlabel('Time (s)')
axs[2].set_ylabel('Acc (m/s^2)')
axs[2].legend()
axs[2].grid(True)

plt.tight_layout()
plt.savefig('accelerometer_3takes_subplots_filtered_vs_raw.png', dpi=150)
plt.show()

# Behavior of a thrown ball max Acceleration with a drop off
# around min acceleration made by athlete is 10 m/s^2 to start the throw
# then it drops almost inmediately to -10 m/s^2.

#Calculate the time passed during the peak and drop
#dt = 1/52 # IMU sample rate 52Hz
#for i in range(150):
 #   if F1[i] > 10:
  #      print("Peak detected at index:", i)
   #     break
#time_peak1 = F1.index(max(F1)) * dt
#time_drop1 = F1.index(min(F1)) * dt
#timelapsed1 = time_drop1 - time_peak1

#time_peak2 = F2.index(max(F2)) * dt
#time_drop2 = F2.index(min(F2)) * dt
#time_peak3 = F3.index(max(F3)) * dt
#time_drop3 = F3.index(min(F3)) * dt

"""

# Launch detection algorithm
# Define thresholds
#MAX_ACCEL_THRESHOLD = 10.0  # m/s^2, threshold to detect the push
#GRAVITY_THRESHOLD = 2.0  # m/s^2, threshold to detect free fall (near zero acceleration)
#STATIC_THRESHOLD = -0.8  # m/s^2, threshold to consider as static (no movement)

#Trajectory data combined with time
#dt = 1/52  # Time step based on IMU sample rate (52 Hz)
#F1 = np.array(F1)
#print(f"Shape of F1: {F1.shape}")
#timestamps = np.arange(len(F1)) * dt
#print(f"Shape of Time Stamps: {timestamps.shape}")
#timestamps = np.array(timestamps)
#trajectory_data = list(zip(timestamps, F1))

#def detect_phases(accels):
    """
    #Detects the indices for initial acceleration and launch.
"""
    initial_accel_index = None
    launch_index = None
    in_push_phase = False

    for i in range(len(accels)):
        accel = accels[i]
        
        # Check for the start of the push (initial acceleration)
        if accel > MAX_ACCEL_THRESHOLD and not in_push_phase:
            initial_accel_index = i
            in_push_phase = True
            
        # Check for the launch (acceleration drops back down)
        if in_push_phase and accel < GRAVITY_THRESHOLD:
            launch_index = i
            break
            
    return initial_accel_index, launch_index

# --- Plotting ---
initial_accel_idx, launch_idx = detect_phases(F1)

plt.figure(figsize=(10, 6))
plt.plot(timestamps, F1, label='Acceleration Magnitude', marker='o', markersize=4)

# Plot and label the static phase
static_time = timestamps[0]
static_accel = F1[0]
plt.annotate('1. Static in Hand', 
             xy=(static_time, static_accel), 
             xytext=(static_time, static_accel + 2),
             arrowprops=dict(facecolor='blue', shrink=0.05))

# Plot and label the initial acceleration phase
if initial_accel_idx is not None:
    initial_accel_time = timestamps[initial_accel_idx]
    initial_accel_val = F1[initial_accel_idx]
    plt.annotate('2. Initial Acceleration', 
                 xy=(initial_accel_time, initial_accel_val), 
                 xytext=(initial_accel_time, initial_accel_val + 5),
                 arrowprops=dict(facecolor='purple', shrink=0.05))
    plt.plot(initial_accel_time, initial_accel_val, 'r^', markersize=10, label='Initial Acceleration')

# Plot and label the thrown (in flight) phase
if launch_idx is not None:
    launch_time = timestamps[launch_idx]
    launch_accel = F1[launch_idx]
    plt.annotate('3. Thrown (In Flight)', 
                 xy=(launch_time, launch_accel), 
                 xytext=(launch_time, launch_accel + 2),
                 arrowprops=dict(facecolor='green', shrink=0.05))
    plt.plot(launch_time, launch_accel, 'go', markersize=10, label='Launch Point')

# Add plot labels and a title
plt.title('Three Phases of a Ball Throw')
plt.xlabel('Time (s)')
plt.ylabel('Acceleration Magnitude (m/s²)')
plt.grid(True)
plt.legend()

plt.show()

    """
# --- End of Launch detection algorithm ---