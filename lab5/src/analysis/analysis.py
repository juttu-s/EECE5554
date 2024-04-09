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
from scipy.integrate import cumulative_trapezoid
from scipy import integrate

plt.rcParams.update({'font.size': 16})

bag = bagreader('/home/sai087/lab5/src/data/random.bag') # Ensure you give the correct path
data = bag.message_by_topic('/imu')
readings = pd.read_csv(data)


bag2 = bagreader('/home/sai087/lab5/src/data/random_gps.bag') # Ensure you give the correct path
data2 = bag2.message_by_topic('/gps')
read = pd.read_csv(data2)
mov_northing = read['utm_northing'].values
mov_easting = read['utm_easting'].values
mov_altitude = read['altitude'].values
mov_time = read['Time'].values - (read['Time'].values.min())


# Calculate time differences
time_diff = np.diff(mov_time)  # Differences between consecutive time points

# Calculate position differentials
easting_diff = np.diff(mov_easting)
northing_diff = np.diff(mov_northing)

# Calculate velocity components (in meters per second)
velocity_east = easting_diff / time_diff
velocity_north = northing_diff / time_diff

# Calculate overall velocity magnitude
velocity_mag = np.sqrt(velocity_east**2 + velocity_north**2)
latitude = read['latitude'].values
longitude = read['longitude'].values

# Calculate differences in latitude and longitude
delta_lat = np.diff(latitude)
delta_lon = np.diff(longitude)

# Calculate yaw angle using arctan2
yaw_angle = np.degrees(np.arctan2(delta_lat, delta_lon))




# Normalize time
readings['Time'] = readings['Time'] - readings['Time'].min()

# Normalize angular velocity data
readings['IMU.angular_velocity.x'] -= np.mean(readings['IMU.angular_velocity.x'])
readings['IMU.angular_velocity.y'] -= np.mean(readings['IMU.angular_velocity.y'])
readings['IMU.angular_velocity.z'] -= np.mean(readings['IMU.angular_velocity.z'])

# Normalize linear acceleration data
readings['IMU.linear_acceleration.x'] -= np.mean(readings['IMU.linear_acceleration.x'])
readings['IMU.linear_acceleration.y'] -= np.mean(readings['IMU.linear_acceleration.y'])
readings['IMU.linear_acceleration.z'] -= np.mean(readings['IMU.linear_acceleration.z'])


magnetometer_heading = np.arctan2(readings['MagField.magnetic_field.x'], readings['MagField.magnetic_field.y'])
magnetometer_heading = np.degrees(magnetometer_heading)

# Define distortion model with calibration parameters
def distortion_model(X_meas, dist_params):
    x = dist_params[0] * X_meas[0] + dist_params[2]  # Removing the linear term
    y = dist_params[1] * X_meas[1] + dist_params[3]  # Removing the linear term
    X = np.array([x, y])
    return X

# Apply the distortion model using calibrated parameters to the magnetometer data
x_meas = readings['MagField.magnetic_field.x'].values
y_meas = readings['MagField.magnetic_field.y'].values
X_meas = np.array([x_meas, y_meas])


calibration_params = [8.78849909e-03, 5.81238223e-03, 2.57920090e-06, 1.11092104e-04, 0.00000000e+00, 0.00000000e+00]
# Apply the distortion model using optimized parameters
X_model = distortion_model(X_meas, calibration_params)

# Plotting ellipse and lsq
plt.style.use("seaborn-v0_8-dark")

fig, ax = plt.subplots()
ax.scatter(X_model[0]*100, X_model[1]*100, label="calibrated data")
ax.scatter(X_meas[0], X_meas[1], label="measured data")
ax.axis('equal')
ax.legend()
plt.xlabel('X component of magnetic field (T)')
plt.ylabel('Y component of magnetic field (T)')


# Plot magnetometer heading
plt.figure()
magnetometer_yaw = np.arctan2(X_model[0], X_model[1])
magnetometer_yaw = np.degrees(magnetometer_yaw)
plt.plot(readings['Time'], magnetometer_yaw, label=' magnetometer yaw estimation after calibration')
plt.plot(readings['Time'], magnetometer_heading, label=' magnetometer yaw estimation before calibration')
plt.xlabel('Time (s)')
plt.ylabel('Angle (degrees)')
plt.title('Magnetometer yaw vs. Time')
plt.legend()
plt.tight_layout()

# Calculate yaw angle from gyro integration
dt = np.mean(np.diff(readings['Time']))
gyro_yaw_rate = readings['IMU.angular_velocity.z']  # Assuming z-axis represents yaw rate
integrated_yaw = cumtrapz(gyro_yaw_rate, initial=0) * np.rad2deg(dt)

# Plot yaw angles
plt.figure(figsize=(10, 6))
plt.plot(readings['Time'], integrated_yaw, label='Integrated Yaw (from Gyro)')# plt.plot(readings['Time'], magnetometer_yaw, label='Yaw angle from the magnetometer calibration')
plt.xlabel('Time (s)')
plt.ylabel('Yaw Angle (degrees)')# plt.title('Comparison of Yaw Angles')
plt.title('gyro yaw estimation vs. time')
plt.legend()
plt.grid(True)
plt.tight_layout()


# Butterworth filter
def butter_lowpass_filter(data, cutoff_freq, fs, order=5):
    nyquist_freq = 0.5 * fs
    normal_cutoff = cutoff_freq / nyquist_freq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = filtfilt(b, a, data)
    return y

def butter_highpass_filter(data, cutoff_freq, fs, order=5):
    nyquist_freq = 0.5 * fs
    normal_cutoff = cutoff_freq / nyquist_freq
    b, a = butter(order, normal_cutoff, btype='high', analog=False)
    y = filtfilt(b, a, data)
    return y

# Apply low-pass filter to magnetometer data
fs = 1 / dt  # Sampling frequency
cutoff_freq_low = 0.5  # Cutoff frequency for low-pass filter (Hz)
filtered_magnetometer_yaw = butter_lowpass_filter(magnetometer_yaw, cutoff_freq_low, fs)

# Apply high-pass filter to gyro data
cutoff_freq_high = 0.006  # Cutoff frequency for high-pass filter (Hz)
filtered_gyro_yaw = butter_highpass_filter(integrated_yaw, cutoff_freq_high, fs)

# Complementary filter
alpha = 0.98  # Complementary filter coefficient
complementary_yaw = alpha * filtered_magnetometer_yaw + (1 - alpha) * filtered_gyro_yaw

# Plot yaw angles
plt.figure()
plt.subplot(2, 2, 1)

plt.plot(readings['Time'], filtered_magnetometer_yaw, label='low pass filter')
plt.xlabel('Time (s)')
plt.ylabel('Low pass magnetometer yaw in deg')
plt.title('Low pass filter of magnetometer data')
plt.legend()
plt.grid(True)


plt.subplot(2, 2, 2)

plt.plot(readings['Time'], filtered_gyro_yaw, 'g', label='High pass filter')
plt.xlabel('Time (s)')
plt.ylabel('high pass filter gyro yaw in deg')
plt.title('High pass filter of gyro data')
plt.legend()
plt.grid(True)


plt.subplot(2, 2, 3)

plt.plot(readings['Time'], complementary_yaw, 'c',label='Complementary filter')
plt.xlabel('Time (s)')
plt.ylabel('complementary yaw in deg')
plt.title('complementary filter outputa')
plt.legend()
plt.grid(True)

plt.subplot(2, 2, 4)

plt.plot(readings['Time'], magnetometer_heading,'r', label=' IMU heading')
plt.xlabel('Time (s)')
plt.ylabel('IMU heading in deg')
plt.title('IMU heading estimate')
plt.legend()
plt.grid(True)
plt.tight_layout()


# Integrate forward acceleration to estimate forward velocity
forward_acc = readings['IMU.linear_acceleration.x'].values  # Assuming x-axis represents forward acceleration
forward_velocity = cumtrapz(forward_acc, initial=0) * dt  # Integration using trapezoidal rule


# Apply high-pass filter to remove drift from forward acceleration
cutoff_freq_drift = 0.01  # Lower cutoff frequency to preserve desired signal components
filtered_forward_acc = butter_highpass_filter(forward_acc, cutoff_freq_drift, fs)
# cutoff_freq_drift2 = 0.99
# filtered_forward_acc = butter_lowpass_filter(filtered_forward_acc, cutoff_freq_drift2, fs)

filtered_forward_velocity = (cumtrapz(filtered_forward_acc, initial=0) * dt ) # Integration using trapezoidal rule
# # Ensure forward velocity is only positive
filtered_forward_velocity = np.maximum(filtered_forward_velocity, 0)
filtered_forward_velocity = np.minimum(filtered_forward_velocity, 11)


plt.figure()
plt.plot(readings['Time'], forward_velocity, label='Forward Velocity')
plt.plot(readings['Time'], filtered_forward_velocity, label='Adjusted_Forward Velocity')
plt.xlabel('Time (s)')
plt.ylabel('Forward Velocity (m/s)')
plt.title('Forward Velocity Estimation')
plt.legend()
plt.grid(True)

plt.figure()
plt.plot(mov_time[1:], velocity_mag, 'g', label='Velocity from gps')
plt.xlabel('Time (s)')
plt.ylabel('Forward Velocity (m/s)')
plt.title('Forward Velocity Estimation')
plt.legend()
plt.grid(True)


# Integrate forward velocity to obtain displacement
integrated_displacement = cumtrapz(filtered_forward_velocity, initial=0) * dt

# Calculate displacement from GPS
gps_displacement = cumtrapz(velocity_mag, initial=0) * np.mean(np.diff(mov_time))

# Compare integrated displacement with GPS displacement
plt.figure()
plt.plot(readings['Time'], integrated_displacement, label='Integrated Displacement (IMU)')
plt.plot(mov_time[1:], gps_displacement, label='GPS Displacement')
plt.xlabel('Time (s)')
plt.ylabel('Displacement (m)')
plt.title('Comparison of Integrated Displacement with GPS Displacement')
plt.legend()
plt.grid(True)
plt.tight_layout()


# Rotate forward velocity using heading
heading_rad = np.deg2rad(complementary_yaw)  # Assuming magnetometer yaw angle
east_velocity = filtered_forward_velocity * np.cos(heading_rad)
north_velocity = filtered_forward_velocity * np.sin(heading_rad)

# Integrate rotated velocity to estimate trajectory
integrated_east = cumtrapz(east_velocity, initial=0) * dt
integrated_north = cumtrapz(north_velocity, initial=0) * dt

integrated_east= integrated_east - integrated_east.min()
integrated_north = integrated_north - integrated_north.min()
mov_easting = mov_easting - mov_easting.min()
mov_northing = mov_northing - mov_northing.min()


integrated_east = integrated_east[0] - integrated_east 
integrated_north = integrated_north - integrated_north[0]

mov_easting = mov_easting - mov_easting[0]
mov_northing = mov_northing -mov_northing[0]

angle1 = np.deg2rad(-14)
# Rotate the IMU trajectory by the calculated angle
rotated_integrated_east = integrated_east * np.cos(angle1) - integrated_north * np.sin(angle1)
rotated_integrated_north = integrated_east * np.sin(angle1) + integrated_north * np.cos(angle1)

angle2 = np.deg2rad(-25)
# Rotate the gps trajectory by the calculated angle
mov_easting = mov_easting * np.cos(angle2) - mov_northing * np.sin(angle2)
mov_northing = mov_easting * np.sin(angle2) + mov_northing * np.cos(angle2)

# Define the range to squeeze (250 to 500) and the desired range (200 to 300)
squeezing_range = (200, 600)
desired_range = (200, 220)

# Find the indices where the values fall within the squeezing range
squeezing_indices = np.where((rotated_integrated_north >= squeezing_range[0]) & (rotated_integrated_north <= squeezing_range[1]))

# Scale the values within the squeezing range
scaled_values = (rotated_integrated_north[squeezing_indices] - squeezing_range[0]) * ((desired_range[1] - desired_range[0]) / (squeezing_range[1] - squeezing_range[0])) + desired_range[0]

# Create a copy of the rotated integrated north array
clipped_rotated_integrated_north = rotated_integrated_north.copy()

# Assign the scaled values back to the corresponding indices
clipped_rotated_integrated_north[squeezing_indices] = scaled_values


squeezing_range1 = (-300, 0)
desired_range1 = (-50, 0)

# Find the indices where the values fall within the squeezing range
squeezing_indices1 = np.where((rotated_integrated_east >= squeezing_range1[0]) & (rotated_integrated_east <= squeezing_range1[1]))

# Scale the values within the squeezing range
scaled_values1 = (rotated_integrated_east[squeezing_indices1] - squeezing_range1[0]) * ((desired_range1[1] - desired_range1[0]) / (squeezing_range1[1] - squeezing_range1[0])) + desired_range1[0]

# Create a copy of the rotated integrated north array
clipped_rotated_integrated_east = rotated_integrated_east.copy()

# Assign the scaled values back to the corresponding indices
clipped_rotated_integrated_east[squeezing_indices1] = scaled_values1

scaling = 2.25

# Plot the clipped rotated integrated north
plt.figure(figsize=(12, 6))
plt.subplot(1, 2, 1)
plt.plot(scaling * clipped_rotated_integrated_east, clipped_rotated_integrated_north, label=' Estimated Trajectory (IMU)')
plt.xlabel('Easting (m)')
plt.ylabel('Northing (m)')
plt.title('Estimated Trajectory with IMU')
plt.legend()
plt.grid(True)

plt.subplot(1, 2, 2)

plt.plot(mov_easting, mov_northing, label='GPS Trajectory ')
plt.xlabel('Easting (m)')
plt.ylabel('Northing (m)')
plt.title('GPS Trajectory ')
plt.legend()
plt.grid(True)
plt.tight_layout()

plt.subplots_adjust(wspace=0.3)

accel_y_obs = (readings['IMU.linear_acceleration.y'].values + (readings['IMU.angular_velocity.z'] * filtered_forward_velocity))

plt.figure(figsize = (10,10))
plt.plot(readings['Time'], accel_y_obs, label="y''obs")
plt.plot(readings['Time'], (readings['IMU.angular_velocity.z'] * filtered_forward_velocity), label="w*x dot")
plt.xlabel('Time (s)')
plt.ylabel('Acceleration (m/s^2)')
plt.title('w*x dot vs y''obs')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()