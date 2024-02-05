#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import roscpp
import serial
import utm
import roslib
import sys
from gps_driver.msg import Customgps
from std_msgs.msg import Header
import time

global line


gps_pub = rospy.Publisher('/gps', Customgps, queue_size=10)
rospy.init_node("gps", anonymous=True)
rate = rospy.Rate(1)

try:
    print("Default Port")
    serial_port = rospy.get_param('~port')
    # serial_port = "/dev/pts/3"
except:
    
    argv=sys.argv
    serial_port = str(argv[1])
    print("Choosing Sys Arg : " , serial_port)


serial_baud = rospy.get_param('~baudrate',4800) 
port = serial.Serial(serial_port,serial_baud)
i=0
f=open("gps_data.txt",'a')

if __name__=='__main__':
    try:
        while(not rospy.is_shutdown()):
                line = port.readline()
                line_arr = str(line)
                line_str = str(line_arr[4:10])
                print(line_str)
                if line_str=='$GPGGA' or str(line_arr[2:8]) == '$GPGGA':
                    
                    msg= Customgps()
                    try:
                        gps_puck_data = line
                        gps_puck_data_arr = str(gps_puck_data)
                        gps_puck_data_str = str(gps_puck_data_arr)
                        gps_puck_data_str = gps_puck_data_str.split(',') 

                        print("GPGGA data:" , gps_puck_data_str)

                        try: 
                     
                            if gps_puck_data_str[3]=='N':
                                msg.Latitude = int(gps_puck_data_str[2][0:2]) + float(gps_puck_data_str[2][2:])/60
                            else: msg.Latitude = -1 * (int(gps_puck_data_str[2][0:2]) + float(gps_puck_data_str[2][2:])/60)

                            if gps_puck_data_str[5]=='E':
                                msg.Longitude = int(gps_puck_data_str[4][0:3]) + float(gps_puck_data_str[4][3:])/60
                            else: msg.Longitude = -1 * (int(gps_puck_data_str[4][0:3]) + float(gps_puck_data_str[4][3:])/60)

                            msg.header.frame_id = 'GPS1_Frame'
                            msg.Altitude = float(gps_puck_data_str[9])
                            UTM_coordinate = utm.from_latlon(msg.Latitude, msg.Longitude)
                            msg.time= float(gps_puck_data_str[1])
                            msg.UTM_easting = UTM_coordinate[0]
                            msg.UTM_northing = UTM_coordinate[1]
                            msg.Zone = UTM_coordinate[2]
                            msg.letter = UTM_coordinate[3]
                            # msg.status = int(gps_puck_data_str[7])


                            utc = float(gps_puck_data_str[1])
                            utc_hrs = utc//10000
                            utc_mint = (utc-(utc_hrs*10000))//100
                            utc_sec = (utc - (utc_hrs*10000) - (utc_mint*100))
                            utc_final_secs = (utc_hrs*3600 + utc_mint*60 + utc_sec)
                            utc_final_nsecs = int((utc_final_secs * (10**7)) % (10**7))
                            print(utc_final_secs , utc_final_nsecs)

                            msg.header.stamp.secs = int(utc_final_secs)
                            msg.header.stamp.nsecs = utc_final_nsecs

                            gps_pub.publish(msg)
                            f.write(str(msg))

                        except Exception as e:
                                print("Error Encountered" , e) 
                                msg.header.frame_id = 'GPS1_Frame'
                                msg.Altitude = 0.0
                                # UTM_coordinate = utm.from_latlon(msg.Latitude, msg.Longitude)
                                msg.time= float(gps_puck_data_str[1])
                                msg.UTM_easting = 0.0
                                msg.UTM_northing = 0.0
                                msg.Zone = 0.0
                                msg.letter = 'Error'

                                utc = float(gps_puck_data_str[1])
                                utc_hrs = utc//10000
                                utc_mint = (utc-(utc_hrs*10000))//100
                                utc_sec = (utc - (utc_hrs*10000) - (utc_mint*100))
                                utc_final_secs = (utc_hrs*3600 + utc_mint*60 + utc_sec)
                                utc_final_nsecs = int((utc_final_secs * (10**7)) % (10**7))
                                # msg.status = int(gps_puck_data_str[7])
                                msg.header.stamp.secs = int(utc_final_secs)
                                msg.header.stamp.nsecs = utc_final_nsecs
                                print(utc_final_secs , utc_final_nsecs)
                                gps_pub.publish(msg)
                                pass  

                    except: 
                        rospy.logwarn("Exception" + line)
                        continue
                rospy.sleep(0.5)
    except rospy.ROSInterruptException:
        port.close()
    
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down gps node...")            
    f.close()


