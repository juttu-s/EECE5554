#!/usr/bin/env python3
import sys
import math as m
import rospy
import serial
from std_msgs.msg import Header
from vn_driver.msg import Vectornav

global line

imu_pub = rospy.Publisher('/imu', Vectornav, queue_size=10)
rospy.init_node("imu_node", anonymous=True)
rate = rospy.Rate(40)

try :
    serial_port = rospy.get_param('~port')
except:
    argv=sys.argv
    serial_port = str(argv[1])

serial_baud = rospy.get_param('~baudrate',115200) 
port = serial.Serial(serial_port,serial_baud)
port.write(b"$VNWRG,07,40*xx")
#imu_data_file=open("imu_data.txt",'a')

def imu_driver():
    while(not rospy.is_shutdown()):
        data = str(port.readline())
        string_data = str(data)
        if "$VNYMR" in data or string_data[4:10] == '$VNYMR' or string_data[2:8] == '$VNYMR':
            msg = Vectornav()
            sensor_data = data.split(',')        
            print("IMU data:", sensor_data)
            
            time = rospy.get_rostime()
            yaw = float(sensor_data[1])
            pitch = float(sensor_data[2])
            roll = float(sensor_data[3])
            mag_dataX = float(sensor_data[4])
            mag_dataY = float(sensor_data[5])
            mag_dataZ = float(sensor_data[6])
            accel_dataX = float(sensor_data[7])
            accel_dataY = float(sensor_data[8])
            accel_dataZ = float(sensor_data[9])
            gyro_dataX = float(sensor_data[10])
            gyro_dataY = float(sensor_data[11])
            gyro_dataZ = float(sensor_data[12][0:9])


            quat_datax = m.sin(roll/2) * m.cos(pitch/2) * m.cos(yaw/2) - m.cos(roll/2) * m.sin(pitch/2) * m.sin(yaw/2)
            quat_datay = m.cos(roll/2) * m.sin(pitch/2) * m.cos(yaw/2) + m.sin(roll/2) * m.cos(pitch/2) * m.sin(yaw/2)
            quat_dataz = m.cos(roll/2) * m.cos(pitch/2) * m.sin(yaw/2) - m.sin(roll/2) * m.sin(pitch/2) * m.cos(yaw/2)
            quat_dataw = m.cos(roll/2) * m.cos(pitch/2) * m.cos(yaw/2) + m.sin(roll/2) * m.sin(pitch/2) * m.sin(yaw/2)

            try:
                msg.Header.stamp.secs = int(time.secs)
                msg.Header.stamp.nsecs = int(time.nsecs)
                msg.Header.frame_id = 'imu1_frame'
                msg.IMU.orientation.x = quat_datax
                msg.IMU.orientation.y = quat_datay
                msg.IMU.orientation.z = quat_dataz
                msg.IMU.orientation.w = quat_dataw
                msg.IMU.angular_velocity.x = gyro_dataX
                msg.IMU.angular_velocity.y = gyro_dataY
                msg.IMU.angular_velocity.z = gyro_dataZ
                msg.IMU.linear_acceleration.x = accel_dataX
                msg.IMU.linear_acceleration.y = accel_dataY
                msg.IMU.linear_acceleration.z = accel_dataZ
                msg.MagField.magnetic_field.x = mag_dataX
                msg.MagField.magnetic_field.y = mag_dataY
                msg.MagField.magnetic_field.z = mag_dataZ
                msg.imu_string = string_data[2 :]
                imu_pub.publish(msg)
                #imu_data_file.write(str(msg))

            except:
                print("Error in data received!")
                msg.Header.stamp.secs = int(time.secs)
                msg.Header.stamp.nsecs = int(time.nsecs)
                msg.Header.frame_id = 'imu1_frame'
                msg.IMU.orientation.x = 0.0
                msg.IMU.orientation.y = 0.0
                msg.IMU.orientation.z = 0.0
                msg.IMU.orientation.w = 0.0
                msg.IMU.angular_velocity.x = 0.0
                msg.IMU.angular_velocity.y = 0.0
                msg.IMU.angular_velocity.z = 0.0
                msg.IMU.linear_acceleration.x = 0.0
                msg.IMU.linear_acceleration.y = 0.0
                msg.IMU.linear_acceleration.z = 0.0
                msg.MagField.magnetic_field.x = 0.0
                msg.MagField.magnetic_field.y = 0.0
                msg.MagField.magnetic_field.z = 0.0
                msg.imu_string = string_data[2 :]
                imu_pub.publish(msg)
                # imu_data_file.write(str(msg))

            else:
                print("Waiting for VNYMR data type")
            

if __name__ == '__main__':
    try:
        imu_driver()
    except rospy.ROSInterruptException:
        pass
        port.close()
    #imu_data_file.close()