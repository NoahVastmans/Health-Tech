import pandas as pd
import numpy as np

# --- 1. Data Loading and Unit Conversion ---
# Setting the sheets to read
sheet_to_read1 = '3 throws Acc take1'
sheet_to_read2 = '3 throws Gyr take1'
sheet_to_read3 = '3 throws Acc take2'
sheet_to_read4 = '3 throws Gyr take2'
sheet_to_read5 = '3 throws Acc take3'
sheet_to_read6 = '3 throws Gyr take3'

# IMU Raw data values to read
columns_to_readAcc = ['X [mg]', 'Y [mg]', 'Z [mg]']
columns_to_readGyr = ['X [dps]', 'Y [dps]', 'Z [dps]']
time_column = 'Time [sec]'

# Read specified columns from the chosen sheet
try:
    Acc1 = pd.read_excel('Data 0904.xlsx', sheet_name=sheet_to_read1, usecols=columns_to_readAcc)
    Gyr1 = pd.read_excel('Data 0904.xlsx', sheet_name=sheet_to_read2, usecols=columns_to_readGyr)
    Time1 = pd.read_excel('Data 0904.xlsx', sheet_name=sheet_to_read1, usecols=[time_column])

    Acc2 = pd.read_excel('Data 0904.xlsx', sheet_name=sheet_to_read3, usecols=columns_to_readAcc)
    Gyr2 = pd.read_excel('Data 0904.xlsx', sheet_name=sheet_to_read4, usecols=columns_to_readGyr)
    Time2 = pd.read_excel('Data 0904.xlsx', sheet_name=sheet_to_read3, usecols=[time_column])

    Acc3 = pd.read_excel('Data 0904.xlsx', sheet_name=sheet_to_read5, usecols=columns_to_readAcc)
    Gyr3 = pd.read_excel('Data 0904.xlsx', sheet_name=sheet_to_read6, usecols=columns_to_readGyr)
    Time3 = pd.read_excel('Data 0904.xlsx', sheet_name=sheet_to_read5, usecols=[time_column])
except Exception as e:
    print(f"Error reading Excel file: {e}")
    exit()

# Conversion factors
mg2ms2 = 9.81 / 1000.0
deg2rad = np.pi / 180.0

def to_ndarrays(Acc_df, Gyr_df, Time_df):
    acc_cols = ['X [mg]', 'Y [mg]', 'Z [mg]']
    gyr_cols = ['X [dps]', 'Y [dps]', 'Z [dps]']

    acc = Acc_df[acc_cols].values.astype(float) * mg2ms2
    gyr = Gyr_df[gyr_cols].values.astype(float) * deg2rad
    
    try:
        times = Time_df[time_column].values.astype(float)
    except Exception:
        sample_rate_hz = 100.0
        times = np.arange(acc.shape[0]) * (1.0 / sample_rate_hz)

    n = min(acc.shape[0], gyr.shape[0], times.shape[0])
    return acc[:n, :], gyr[:n, :], times[:n]

acc1, gyr1, t1 = to_ndarrays(Acc1, Gyr1, Time1)
acc2, gyr2, t2 = to_ndarrays(Acc2, Gyr2, Time2)
acc3, gyr3, t3 = to_ndarrays(Acc3, Gyr3, Time3)

# --- 2. Manual Madgwick Filter and State Machine ---

# Manual Quaternion Math functions from your C code logic
def quat_mult(q1, q2):
    return np.array([
        q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3],
        q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2],
        q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1],
        q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]
    ])

def quat_normalize(q):
    return q / np.linalg.norm(q)

def quat_conjugate(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])

def quat_rotate(q, v):
    q_v = np.array([0, v[0], v[1], v[2]])
    q_out = quat_mult(quat_mult(q, q_v), quat_conjugate(q))
    return q_out[1:]

def process_take_arrays(acc_arr, gyr_arr, times, take_number):
    print(f"\n--- Processing Take {take_number} ---")
    
    # Initialize state variables and quaternion for each take
    is_on_ground = True
    is_launched = False
    is_in_free_fall = False
    vz = 0.0
    vz_previous = 0.0
    q_est = np.array([1.0, 0.0, 0.0, 0.0]) # Quaternion starts at no rotation
    
    delta_time = np.mean(np.diff(times)) if len(times) > 1 else 0.01

    # Thresholds (tune these)
    LAUNCH_ACCEL_THRESHOLD = 20.0
    FREEFALL_ACCEL_THRESHOLD = 1.5
    LANDING_ACCEL_THRESHOLD = 15.0

    for i in range(len(acc_arr)):
        accel_i = acc_arr[i]
        gyro_i = gyr_arr[i]

        # Manual Madgwick Filter Step
        # This is a simplified Madgwick step to get the linear acceleration
        gyro_quat_rate = np.array([0, gyro_i[0], gyro_i[1], gyro_i[2]])
        q_est_rate = 0.5 * quat_mult(q_est, gyro_quat_rate)
        q_est += q_est_rate * delta_time
        q_est = quat_normalize(q_est)
        
        # Calculate linear acceleration by removing gravity
        gravity_vector = np.array([0, 0, 9.81])
        rotated_gravity = quat_rotate(q_est, gravity_vector)
        linear_accel = accel_i - rotated_gravity
        
        # Update vertical velocity (z-axis component)
        vz_previous = vz
        vz += linear_accel[2] * delta_time

        # State Machine Logic
        if is_on_ground and np.linalg.norm(linear_accel) > LAUNCH_ACCEL_THRESHOLD:
            is_on_ground = False
            is_launched = True
            vz = 0.0
            print(f"[{times[i]:.2f}s] LAUNCH DETECTED")

        elif is_launched and np.linalg.norm(linear_accel) < FREEFALL_ACCEL_THRESHOLD:
            is_launched = False
            is_in_free_fall = True
            print(f"[{times[i]:.2f}s] FREE FALL / APEX PHASE")

        if is_in_free_fall:
            if vz < 0.0 and vz_previous >= 0.0:
                print(f"[{times[i]:.2f}s] APEX REACHED")
        
        if is_in_free_fall and np.linalg.norm(linear_accel) > LANDING_ACCEL_THRESHOLD:
            is_in_free_fall = False
            is_on_ground = True
            print(f"[{times[i]:.2f}s] LANDING DETECTED")
            print("--------------------------------------------------")

# --- 3. Run the Algorithm for Each Take ---
process_take_arrays(acc1, gyr1, t1, 1)
process_take_arrays(acc2, gyr2, t2, 2)
process_take_arrays(acc3, gyr3, t3, 3)