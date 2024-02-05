import numpy as np 
import matplotlib.pyplot as plt
import pandas as pd

# Load data from CSV files
open_df = pd.read_csv('open_stationary.csv')
open_northing = open_df['field.UTM_northing'].values
open_easting = open_df['field.UTM_easting'].values
open_altitude = open_df['field.Altitude'].values
open_time = open_df['%time'].values
occ_df = pd.read_csv('occluded_stationary.csv')
occ_northing = occ_df['field.UTM_northing'].values
occ_easting = occ_df['field.UTM_easting'].values
occ_altitude = occ_df['field.Altitude'].values
occ_time = occ_df['%time'].values
mov_df = pd.read_csv('Moving.csv')
mov_northing = mov_df['field.UTM_northing'].values
mov_easting = mov_df['field.UTM_easting'].values
mov_altitude = mov_df['field.Altitude'].values
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

occ_centroid_northing = np.mean(occ_northing) 
occ_centroid_easting = np.mean(occ_easting)

# Subtract centroids
open_northing -= open_centroid_northing
open_easting -= open_centroid_easting

occ_northing -= occ_centroid_northing
occ_easting -= occ_centroid_easting

# Calculate Euclidean distances
open_distances = np.sqrt((open_northing**2) + (open_easting**2))
occ_distances = np.sqrt((occ_northing**2) + (occ_easting**2))

#print centroid values
print('Open_stationary_centroid_Northing %d , Open_stationary_centroid_Easting %s' % (open_centroid_northing, open_centroid_easting))
print('Occlusion_stationary_centroid_Northing %d , Occlusion_stationary_centroid_Easting %s' % (occ_centroid_northing, occ_centroid_easting))

fig1, (ax1) = plt.subplots()

# Northing vs Easting subplot
ax1.scatter(open_northing, open_easting, label='Open') 
ax1.scatter(occ_northing, occ_easting, label='Occluded')
ax1.set_xlabel('Northing (m)')
ax1.set_ylabel('Easting (m)')
ax1.set_title('Northing vs Easting for open and occlusion_stationary')
ax1.legend()
ax1.grid(True)

fig2, (ax2) = plt.subplots()
# Altitude vs Time subplot  
ax2.scatter(open_time, open_altitude, label='Open')
ax2.scatter(occ_time, occ_altitude, label='Occluded') 
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Altitude (m)')  
ax2.legend()
ax2.grid(True)
ax2.set_title('Altitude vs Time subplot for open and occlusion_stationary')


# Histogram plots for position
fig3, (ax3, ax4) = plt.subplots(1, 2, figsize=(12, 6))

ax3.hist(open_distances, bins=20, color='blue', alpha=0.7, label='Open')
ax3.set_xlabel('Euclidean Distance from Centroid (m)')
ax3.set_ylabel('Frequency')
ax3.legend()
ax3.set_title('Histogram plots for Open_stationary')
ax3.grid(True)

ax4.hist(occ_distances, bins=20, color='orange', alpha=0.7, label='Occluded')
ax4.set_xlabel('Euclidean Distance from Centroid (m)')
ax4.set_ylabel('Frequency')
ax4.legend()
ax4.set_title('Histogram plots for Occlusion_stationary')
ax4.grid(True)



fig4, (ax1) = plt.subplots()

# Northing Moving data vs Easting subplot
ax1.scatter(mov_northing, mov_easting, label='Moving_data') 
ax1.set_xlabel('Northing (m)')
ax1.set_ylabel('Easting (m)')
ax1.legend()
ax1.set_title('Moving data vs Easting subplot for Moving data')
ax1.grid(True)

fig5, (ax2) = plt.subplots()
# Moving data Altitude vs Time subplot  
ax2.scatter(mov_time, mov_altitude, label='Moving_data')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Altitude (m)')  
ax2.legend()
ax2.set_title('Altitude vs Time subplot for Moving data')
ax2.grid(True)

plt.tight_layout()
plt.show()