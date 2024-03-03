import csv
import numpy as np
import matplotlib.pyplot as plt
from allantools import oadev

# Load sensor data from CSV file and downsample
downsample_factor = 10  # Adjust as needed
sensor_data = {'Gyro_X': [], 'Gyro_Y': [], 'Gyro_Z': [], 'Accel_X': [], 'Accel_Y': [], 'Accel_Z': []}
try:
    with open('sensor_data.csv', mode='r') as file:
        reader = csv.DictReader(file)
        for i, row in enumerate(reader):
            if i % downsample_factor == 0:
                for key in sensor_data.keys():
                    sensor_data[key].append(float(row[key]))
except FileNotFoundError:
    print("Error: File not found.")
    exit(1)
except Exception as e:
    print(f"An error occurred: {e}")
    exit(1)

# Convert data to numpy arrays
sensor_data_array = {key: np.array(value) for key, value in sensor_data.items()}

# Calculate Allan variance for gyro and accelerometer data
allan_variances = {}
for axis, data in sensor_data_array.items():
    taus, avar, _, _ = oadev(data, rate=1, data_type='freq', taus='all')
    allan_variances[axis] = (taus, avar)

# Analyze Allan variance plots to determine B, N, and K
results = {}
for axis, (taus, avar) in allan_variances.items():
    # Find the index of the minimum Allan variance
    min_index = np.argmin(avar)
    # Bias instability (B) is the value at the local minimum Allan variance
    B = np.sqrt(avar[min_index])
    # Angle random walk (N) at tau = 1 s
    N_index = np.where(taus == 1)[0][0]
    N = np.sqrt(avar[N_index])
    # Rate random walk (K) as given by the slope after the local minimum
    slope = (avar[min_index+1] - avar[min_index]) / (taus[min_index+1] - taus[min_index])
    K = np.sqrt(slope)
    results[axis] = {'Bias Instability (B)': B, 'Angle Random Walk (N)': N, 'Rate Random Walk (K)': K}

# Print results
for axis, values in results.items():
    print(f"Results for {axis}:")
    for param, value in values.items():
        print(f"{param}: {value}")
    print()
    
plt.figure(figsize=(12, 8))
for axis, (taus, avar) in allan_variances.items():
    if 'Gyro' in axis:
        plt.loglog(taus, avar, label=f'{axis}')
        # Marking key points
        min_index = np.argmin(avar)
        plt.scatter(taus[min_index], avar[min_index], color='red', marker='o')  # local minimum
        slope_index = min_index + 1
        plt.scatter(taus[slope_index], avar[slope_index], color='green', marker='o')  # slope calculation point
        # Annotate B, N, and K
        plt.annotate(f'B: {results[axis]["Bias Instability (B)"]:.2e}', xy=(taus[min_index], avar[min_index]),
                     xytext=(20, -30), textcoords='offset points', arrowprops=dict(facecolor='black', arrowstyle='->'))
        plt.annotate(f'N: {results[axis]["Angle Random Walk (N)"]:.2e}', xy=(taus[1], avar[1]), xytext=(-40, 20),
                     textcoords='offset points', arrowprops=dict(facecolor='black', arrowstyle='->'))
        plt.annotate(f'K: {results[axis]["Rate Random Walk (K)"]:.2e}', xy=(taus[slope_index], avar[slope_index]),
                     xytext=(-40, 20), textcoords='offset points', arrowprops=dict(facecolor='black', arrowstyle='->'))
plt.xlabel('Tau (s)')
plt.ylabel('Allan Variance')
plt.title('Allan Variance Plot for Gyro Data')
plt.legend()
plt.grid(True)
plt.xlim(1e0, 1e5)  # Set x-axis limit from 10^0 to 10^5
plt.ylim(1e-7, 1e-1)  # Set y-axis limit from 10^-6 to 10^-1
plt.show()

# Plot Allan variance for accelerometer data
plt.figure(figsize=(12, 8))
for axis, (taus, avar) in allan_variances.items():
    if 'Accel' in axis:
        plt.loglog(taus, avar, label=f'{axis}')
        # Marking key points
        min_index = np.argmin(avar)
        plt.scatter(taus[min_index], avar[min_index], color='red', marker='o')  # local minimum
        slope_index = min_index + 1
        plt.scatter(taus[slope_index], avar[slope_index], color='green', marker='o')  # slope calculation point
        # Annotate B, N, and K
        plt.annotate(f'B: {results[axis]["Bias Instability (B)"]:.2e}', xy=(taus[min_index], avar[min_index]),
                     xytext=(20, -30), textcoords='offset points', arrowprops=dict(facecolor='black', arrowstyle='->'))
        plt.annotate(f'N: {results[axis]["Angle Random Walk (N)"]:.2e}', xy=(taus[1], avar[1]), xytext=(-40, 20),
                     textcoords='offset points', arrowprops=dict(facecolor='black', arrowstyle='->'))
        plt.annotate(f'K: {results[axis]["Rate Random Walk (K)"]:.2e}', xy=(taus[slope_index], avar[slope_index]),
                     xytext=(-40, 20), textcoords='offset points', arrowprops=dict(facecolor='black', arrowstyle='->'))
plt.xlabel('Tau (s)')
plt.ylabel('Allan Variance')
plt.title('Allan Variance Plot for Accelerometer Data')
plt.legend()
plt.grid(True)
plt.xlim(1e0, 1e5)  # Set x-axis limit from 10^0 to 10^5
plt.ylim(1e-4, 1e-1)  # Set y-axis limit from 10^-6 to 10^-1
plt.show()