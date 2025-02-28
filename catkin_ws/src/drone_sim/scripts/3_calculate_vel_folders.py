import pandas as pd
import numpy as np
from scipy.signal import medfilt
import math
import os

def quaternion_to_yaw_pitch_roll(q0, q1, q2, q3):
    """
    Convert quaternion to yaw, pitch, roll (in radians).
    
    Parameters:
    quat: list or tuple of 4 elements [q0, q1, q2, q3]
    
    Returns:
    yaw, pitch, roll: tuple of floats in radians
    """
    # q0, q1, q2, q3 = quat

    # Roll (rotation around x-axis)
    sinr_cosp = 2 * (q0 * q1 + q2 * q3)
    cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (rotation around y-axis)
    sinp = 2 * (q0 * q2 - q3 * q1)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (rotation around z-axis)
    siny_cosp = 2 * (q0 * q3 + q1 * q2)
    cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return yaw, pitch, roll

def list_folders(directory):
    """List all folders in the specified directory."""
    try:
        # Get all items in the directory
        items = os.listdir(directory)
        
        # Filter and list only folders
        folders = [item for item in items if os.path.isdir(os.path.join(directory, item))]

        print("Folders in", directory, ":")
        for folder in folders:
            print(folder)

        return folders
    except Exception as e:
        print("Error:", str(e))
        return []

# Function to apply median filter to columns x, y, z, yaw
def apply_median_filter(df, kernel_size=3):
    # Apply median filter on each of the columns (x, y, z, yaw)
    df['x'] = medfilt(df['x'], kernel_size=kernel_size)
    df['y'] = medfilt(df['y'], kernel_size=kernel_size)
    df['z'] = medfilt(df['z'], kernel_size=kernel_size)
    df['yaw'] = medfilt(df['yaw'], kernel_size=kernel_size)
    
    return df

def normalize_vector(vx, vy, vz):
    """
    Normalize a velocity vector (vx, vy, vz) to a unit vector.

    Parameters:
        vx (float): Velocity in x-direction.
        vy (float): Velocity in y-direction.
        vz (float): Velocity in z-direction.

    Returns:
        tuple: Normalized vector (nx, ny, nz).
    """
    # Calculate the magnitude of the vector
    magnitude = math.sqrt(vx**2 + vy**2 + vz**2)

    # Avoid division by zero
    if magnitude == 0:
        raise ValueError("Cannot normalize a zero vector")

    # Normalize each component
    nx = vx / magnitude
    ny = vy / magnitude
    nz = vz / magnitude

    return nx, ny, nz

# Function to calculate local velocity vector and change in yaw between two points
def calculate_local_velocity(df):
    velocity_vectors = []
    
    # Iterate over the DataFrame to calculate velocity between points i and i+5
    for i in range(0, len(df) - step, 1):
        # Get the difference in position between points i and i+5
        dx = df.loc[i+step, 'x'] - df.loc[i, 'x']
        dy = df.loc[i+step, 'y'] - df.loc[i, 'y']
        dz = df.loc[i+step, 'z'] - df.loc[i, 'z']

        qx = df.loc[i+step, 'qx'] - df.loc[i, 'qx']
        qy = df.loc[i+step, 'qy'] - df.loc[i, 'qy']
        qz = df.loc[i+step, 'qz'] - df.loc[i, 'qz']
        qw = df.loc[i+step, 'qw'] - df.loc[i, 'qw']

        # Get yaw of point i
        yaw_i, pitch, roll = quaternion_to_yaw_pitch_roll(df.loc[i, 'qx'], df.loc[i, 'qy'],df.loc[i, 'qz'], df.loc[i, 'qw'])
        yaw_ip1, pitch, roll = quaternion_to_yaw_pitch_roll(df.loc[i+step, 'qx'], df.loc[i+step, 'qy'],df.loc[i+step, 'qz'], df.loc[i+step, 'qw'])
        
        # Change in yaw (yaw difference between points i and i+5)
        delta_yaw = yaw_ip1 - yaw_i
        delta_yaw = 0
        
        # Filter out if delta_yaw > pi/2
        if abs(delta_yaw) > np.pi/2:
            continue  # Skip this row if the condition is met
        
        # Rotation matrix for yaw angle (2D rotation, assuming yaw is in radians)
        cos_yaw = np.cos(yaw_i)
        sin_yaw = np.sin(yaw_i)
        
        # Rotate the velocity vector into the local frame
        local_velocity_x = cos_yaw * dx + sin_yaw * dy
        local_velocity_y = sin_yaw * dx + cos_yaw * dy
        local_velocity_z = dz  # Assuming no rotation for the z-axis

        norm_vx, norm_vy, norm_vz = normalize_vector(local_velocity_x, local_velocity_y, local_velocity_z)
        
        # Store the local velocity vector, coordinates, and change in yaw
        velocity_vectors.append({
            'frame_id': df.loc[i, 'frame_id'], 
            'x': df.loc[i, 'x'], 
            'y': df.loc[i, 'y'], 
            'z': df.loc[i, 'z'], 
            'local_vx': local_velocity_x, 
            'local_vy': local_velocity_y, 
            'local_vz': local_velocity_z,
            'delta_yaw': delta_yaw
        })
    
    # Return a DataFrame with the velocity vectors and coordinates
    velocity_df = pd.DataFrame(velocity_vectors)
    
    return velocity_df

# Example usage
directory_path = "/home/sim/ardupilot_docker/to_git/ardupilot_docker/recorded_dataset/Reasoning/Digits/2025-02-27 08:10:36.881087/"
folders = list_folders(directory_path)

step = 1

for folder in folders:
    # file_path = directory_path + folder  + "/"
    print(directory_path)

    file_name = "data.csv"
    df = pd.read_csv(directory_path + file_name)

    # Apply median filter to smooth data before calculating velocities
    # df_filtered = apply_median_filter(df)

    # Calculate local velocity vectors and change in yaw
    local_velocity_df = calculate_local_velocity(df)

    # Save the DataFrame to CSV
    local_velocity_df.to_csv(directory_path + 'local_velocity_norm_with_yaw_' + str(step) + '.csv', index=False)

    # Print the resulting DataFrame to verify
    # print(local_velocity_df)