import math
import csv
import statistics
from bagpy import bagreader
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import pandas as pd
from scipy.optimize import least_squares
from scipy.integrate import cumtrapz
from scipy.signal import butter, filtfilt


plt.rcParams.update({'font.size': 16})


readings = pd.read_csv('/home/sai087/lab5/src/data/imu.csv')

# Normalize time
readings['Time'] = readings['Time'] - readings['Time'].min()

# Normalize angular velocity data
readings['IMU.angular_velocity.x'] -= np.mean(readings['IMU.angular_velocity.x'])
readings['IMU.angular_velocity.y'] -= np.mean(readings['IMU.angular_velocity.y'])
readings['IMU.angular_velocity.z'] -= np.mean(readings['IMU.angular_velocity.z'])

# readings['MagField.magnetic_field.x'] = readings['MagField.magnetic_field.x'] - readings['MagField.magnetic_field.x'].min()
# readings['MagField.magnetic_field.y'] = readings['MagField.magnetic_field.y'] - readings['MagField.magnetic_field.y'].min()
# readings['MagField.magnetic_field.z'] = readings['MagField.magnetic_field.z'] - readings['MagField.magnetic_field.z'].min()

magnetometer_heading = np.arctan2(readings['MagField.magnetic_field.x'], readings['MagField.magnetic_field.y'])
magnetometer_heading = np.degrees(magnetometer_heading)

# Use mean of the data as an initial guess for the distortion parameters
initial_guess = [0, 1, 0, 1, 0, 0]  # Adjusted initial guess for better circular fit

def distortion_model(X_meas, dist_params):
    x = dist_params[0] * X_meas[0] + dist_params[2] 
    y = dist_params[1] * X_meas[1] + dist_params[3] 
    X = np.array([x, y])
    return X

# Function to calculate residuals
def residual(p, X_mag, X_meas):
    return (X_mag - distortion_model(X_meas, p)).flatten()

# Prepare data for optimization
x_meas = readings['MagField.magnetic_field.x'].values
y_meas = readings['MagField.magnetic_field.y'].values
X_meas = np.array([x_meas, y_meas])

x_mag = readings['IMU.angular_velocity.x'].values
y_mag = readings['IMU.angular_velocity.y'].values
X_mag = np.array([x_mag, y_mag])

lsq_min = least_squares(residual, initial_guess, args=(X_mag, X_meas), method='trf', max_nfev=10000)

print("Least square fitting values are: ")
print(lsq_min.x)


# Apply the distortion model using optimized parameters
X_model = distortion_model(X_meas, lsq_min.x)

# Plotting ellipse and lsq
plt.style.use("seaborn-v0_8-dark")

fig, ax = plt.subplots()
ax.scatter(X_model[0]*100, X_model[1]*100, label="calibrated data")
ax.scatter(X_meas[0], X_meas[1], label="measured data")
ax.axis('equal')
ax.legend()
plt.xlabel('X component of magnetic field (Guass)')
plt.ylabel('Y component of magnetic field (Guass)')
plt.show()