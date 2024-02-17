import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# Load data from CSV files
open_df = pd.read_csv('Open_RTK.csv')
open_northing = open_df['field.utm_northing'].values
open_easting = open_df['field.utm_easting'].values
open_latitude = open_df['field.latitude'].values
open_longitude = open_df['field.longitude'].values
open_altitude = open_df['field.altitude'].values
open_time = open_df['%time'].values
occ_df = pd.read_csv('Occluded_RTK.csv')
occ_northing = occ_df['field.utm_northing'].values
occ_easting = occ_df['field.utm_easting'].values
occ_latitude = occ_df['field.latitude'].values
occ_longitude = occ_df['field.longitude'].values
occ_altitude = occ_df['field.altitude'].values
occ_time = occ_df['%time'].values
mov_df = pd.read_csv('Walking_RTK.csv')
mov_northing = mov_df['field.utm_northing'].values
mov_easting = mov_df['field.utm_easting'].values
mov_altitude = mov_df['field.altitude'].values
mov_time = mov_df['%time'].values

# Subtract min time
open_time = open_time - min(open_time)
occ_time = occ_time - min(occ_time)
mov_time = mov_time - min(mov_time)

# Subtract minimum coordinates
mov_northing = mov_northing - min(mov_northing)
mov_easting = mov_easting - min(mov_easting)

# Calculate centroids
open_centroid_northing = np.mean(open_northing)
open_centroid_easting = np.mean(open_easting)
open_centroid_latitude = np.mean(open_latitude)
open_centroid_longitude = np.mean(open_longitude)
occ_centroid_northing = np.mean(occ_northing) 
occ_centroid_easting = np.mean(occ_easting)
occ_centroid_latitude = np.mean(occ_latitude)
occ_centroid_longitude = np.mean(occ_longitude)
# Subtract centroids
open_northing -= open_centroid_northing
open_easting -= open_centroid_easting
occ_northing -= occ_centroid_northing
occ_easting -= occ_centroid_easting

# Calculate Euclidean distances
open_distances = np.sqrt((open_northing**2) + (open_easting**2))
occ_distances = np.sqrt((occ_northing**2) + (occ_easting**2))

# Calculate RMSE for open stationary data
open_rmse = np.sqrt(np.mean((open_northing**2)+(open_easting**2)))

# Calculate RMSE for occluded stationary data
occ_rmse= np.sqrt(np.mean((occ_northing**2)+(occ_easting**2)))

# Compute the squared error for latitude and longitude
open_longitude_error = (open_longitude - open_centroid_longitude) ** 2
open_latitude_error = (open_latitude - open_centroid_latitude) ** 2

occ_longitude_error = (occ_longitude - occ_centroid_longitude) ** 2
occ_latitude_error = (occ_latitude - occ_centroid_latitude) ** 2
# Compute the mean squared error
open_longitude_mse = np.mean(open_longitude_error)
open_latitude_mse = np.mean(open_latitude_error)

occ_longitude_mse = np.mean(occ_longitude_error)
occ_latitude_mse = np.mean(occ_latitude_error)

# Compute the root mean squared error
open_longitude_rmse = np.sqrt(open_longitude_mse)
open_latitude_rmse = np.sqrt(open_latitude_mse)

occ_longitude_rmse = np.sqrt(occ_longitude_mse)
occ_latitude_rmse = np.sqrt(occ_latitude_mse)

# Print the results
print("Open RTK Longitude RMSE:", open_longitude_rmse)
print("Open RTK Latitude RMSE:", open_latitude_rmse)

print("Occ RTK Longitude RMSE:", occ_longitude_rmse)
print("Occ RTK Latitude RMSE:", occ_latitude_rmse)

# Print RMSE values
print("RMSE for open stationary :", open_rmse)

print("RMSE for occlusion stationary:", occ_rmse)


# Calculate the error from the line of best fit
# Fit a line of best fit
coefficients = np.polyfit(mov_northing, mov_easting, 1)
poly_line = np.poly1d(coefficients)

# Calculate the predicted values
predicted_easting = poly_line(mov_northing)

# Calculate the error
error = np.mean(np.abs(predicted_easting - mov_easting))
print("Mean absolute error from the line of best fit:", error)

# Plotting
fig1, ax1 = plt.subplots()

# Northing vs Easting subplot
ax1.scatter(open_northing, open_easting, label='Open_RTK') 
ax1.scatter(occ_northing, occ_easting, label='Occluded_RTK')
ax1.set_xlabel('Northing centered (m)')
ax1.set_ylabel('Easting centered (m)')
ax1.set_title('Northing centered vs Easting centered for Open_RTK and Occluded_RTK')
ax1.text(0.2, 0.3, 'Open Stationary Centroid\nNorthing: {}\nEasting: {}'.format(open_centroid_northing, open_centroid_easting), fontsize=8, ha='center', transform=ax1.transAxes)
ax1.text(0.2, 0.2, 'Occlusion Stationary Centroid\nNorthing: {}\nEasting: {}'.format(occ_centroid_northing, occ_centroid_easting), fontsize=8, ha='center', transform=ax1.transAxes)
ax1.legend()
ax1.grid(True)

fig2, ax2 = plt.subplots()

# Altitude vs Time subplot
ax2.plot(open_time, open_altitude, label='Open_RTK', linewidth=1.5)
ax2.plot(occ_time, occ_altitude, label='Occluded_RTK', linewidth=1.5)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Altitude (m)')
ax2.legend()
ax2.grid(True)
ax2.set_title('Altitude vs Time for Open_RTK and Occluded_RTK')

# Histogram plots for position
fig3, (ax3, ax4) = plt.subplots(1, 2, figsize=(12, 6))

ax3.hist(open_distances, bins=20, color='blue', alpha=0.7, label='Open_RTK')
ax3.set_xlabel('Euclidean Distance from Centroid (m)')
ax3.set_ylabel('Frequency')
ax3.legend()
ax3.set_title('Histogram plots for Open_RTK_stationary')
ax3.grid(True)

ax4.hist(occ_distances, bins=20, color='orange', alpha=0.7, label='Occluded_RTK')
ax4.set_xlabel('Euclidean Distance from Centroid (m)')
ax4.set_ylabel('Frequency')
ax4.legend()
ax4.set_title('Histogram plots for Occlusion_RTK_stationary')
ax4.grid(True)

# Plotting walking RTK data with offset and best fit line
fig4, ax5 = plt.subplots()
ax5.scatter(mov_northing, mov_easting, label='Walking_RTK_data') 

# Plot the best fit line
best_fit_line_x = np.linspace(min(mov_northing), max(mov_northing), 100)
best_fit_line_y = poly_line(best_fit_line_x)
ax5.plot(best_fit_line_x, best_fit_line_y, color='red', label='Best Fit Line')

ax5.set_xlim(min(mov_northing) + 10, max(mov_northing))
ax5.set_ylim(min(mov_easting) + 10, max(mov_easting))
ax5.set_xlabel('Northing_centered (m)')
ax5.set_ylabel('Easting_centered (m)')
ax5.legend()
ax5.set_title('Northing_centered vs Easting_centered for Walking_RTK_data with Best Fit Line')
ax5.grid(True)

# Second Figure: Altitude vs Time
fig5, ax6 = plt.subplots()
ax6.plot(mov_time, mov_altitude, label='Walking_RTK', linewidth=1.5)
ax6.set_xlabel('Time (s)')
ax6.set_ylabel('Altitude (m)')  
ax6.set_xlim(0, 7*10**10)
ax6.legend()
ax6.set_title('Altitude vs Time for Moving data')
ax6.grid(True)

plt.tight_layout()
plt.show()
