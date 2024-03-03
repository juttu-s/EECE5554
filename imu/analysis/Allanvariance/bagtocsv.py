import csv
import pandas as pd
from bagpy import bagreader

bag = bagreader('/home/sai087/EECE5554/imu/data/LocationC.bag')
file = bag.message_by_topic('/vectornav')
readings = pd.read_csv(file)

gyro_dataX_list = []
gyro_dataY_list = []
gyro_dataZ_list = []
accel_dataX_list = []
accel_dataY_list = []
accel_dataZ_list = []

for index, row in readings.iterrows():
    if isinstance(row['data'], str):  # Check if 'data' is a string
        sensor_data = row['data'].split(',')
        if len(sensor_data) >= 13:  # Ensure that 'sensor_data' has at least 13 elements
            try:
                gyro_dataX = float(sensor_data[10].split('*')[0])
                gyro_dataY = float(sensor_data[11].split('*')[0])
                gyro_dataZ = float(sensor_data[12].split('*')[0])
                accel_dataX = float(sensor_data[7].split('*')[0])
                accel_dataY = float(sensor_data[8].split('*')[0])
                accel_dataZ = float(sensor_data[9].split('*')[0])

                # Append the values to the respective lists
                gyro_dataX_list.append(gyro_dataX)
                gyro_dataY_list.append(gyro_dataY)
                gyro_dataZ_list.append(gyro_dataZ)
                accel_dataX_list.append(accel_dataX)
                accel_dataY_list.append(accel_dataY)
                accel_dataZ_list.append(accel_dataZ)

            except ValueError:
                print("Error converting data to float for index:", index)
        else:
            print("Data not complete for index:", index)
    else:
        print("Data is not a string for index:", index)

# Write the gyro and acceleration data to the CSV file
with open('sensor_data.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['Gyro_X', 'Gyro_Y', 'Gyro_Z', 'Accel_X', 'Accel_Y', 'Accel_Z'])  
    for gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z in zip(gyro_dataX_list, gyro_dataY_list, gyro_dataZ_list, accel_dataX_list, accel_dataY_list, accel_dataZ_list):
        writer.writerow([gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z])

# # Print the lists at the end
# print("Gyro X list:", gyro_dataX_list)
# print("Gyro Y list:", gyro_dataY_list)
# print("Gyro Z list:", gyro_dataZ_list)
# print("Accel X list:", accel_dataX_list)
# print("Accel Y list:", accel_dataY_list)
# print("Accel Z list:", accel_dataZ_list)