#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import roscpp
import serial
import utm
import roslib
import sys
from gps_driver.msg import Customrtk
from std_msgs.msg import Header
import time

global line


gps_pub = rospy.Publisher('/gps', Customrtk, queue_size=10)
rospy.init_node('gps', anonymous=True)
rate = rospy.Rate(10)

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

if __name__=='__main__':
    try:
        while(not rospy.is_shutdown()):
                line = port.readline()
                line_arr = str(line)
                line_str = str(line_arr[4:10])
                #print(line_str)
                if line_str=='$GNGGA' or str(line_arr[2:8]) == '$GNGGA':
                    
                    msg= Customrtk()
                    try:
                        gps_puck_data = line
                        gps_puck_data_arr = str(gps_puck_data)
                        gps_puck_data_str = str(gps_puck_data_arr)
                        gps_puck_data_str = gps_puck_data_str.split(',') 

                        print("GNGGA data:" , gps_puck_data_str)

                        try: 
                     
                            if gps_puck_data_str[3]=='N':
                                msg.latitude = int(gps_puck_data_str[2][0:2]) + float(gps_puck_data_str[2][2:])/60
                            else: msg.latitude = -1 * (int(gps_puck_data_str[2][0:2]) + float(gps_puck_data_str[2][2:])/60)

                            if gps_puck_data_str[5]=='E':
                                msg.longitude = int(gps_puck_data_str[4][0:3]) + float(gps_puck_data_str[4][3:])/60
                            else: msg.longitude = -1 * (int(gps_puck_data_str[4][0:3]) + float(gps_puck_data_str[4][3:])/60)

                            msg.header.frame_id = 'GPS1_Frame'
                            msg.altitude = float(gps_puck_data_str[9])
                            utm_coordinate = utm.from_latlon(msg.latitude, msg.longitude)
                            #msg.time= float(gps_puck_data_str[1])
                            msg.utm_easting = utm_coordinate[0]
                            msg.utm_northing = utm_coordinate[1]
                            msg.zone = utm_coordinate[2]
                            msg.letter = utm_coordinate[3]
                            msg.hdop = float(gps_puck_data_str[8])
                            msg.fix_quality = int(gps_puck_data_str[6])
                            msg.gngga_read = str(line)[2:]
                            # msg.status = int(gps_puck_data_str[7])


                            utc = (gps_puck_data_str[1])
                            hours = int(utc[:2])
                            minutes = int(utc[2:4])
                            seconds = int(utc[4:6])
                            milliseconds = float(utc[7:])
                            
                            utcinSecs = hours * 3600 + minutes * 60 + seconds + milliseconds / 1000.0
                            
                            TimeSinceEpoch = time.time() # Get time since epoch
                            TimeSinceEpochBOD = TimeSinceEpoch - (TimeSinceEpoch % 86400) # Get time since epoch at the beginning of the day (86400 seconds in a day)
                            
                            CurrentTime = TimeSinceEpochBOD + utcinSecs
                            utc_final_secs = int(CurrentTime) # Total seconds
                            utc_final_nsecs = int((CurrentTime - utc_final_secs) * 10**9) # Remaining nanoseconds
                            print(utc_final_secs , utc_final_nsecs)
                            msg.header.stamp.secs = int(utc_final_secs)
                            msg.header.stamp.nsecs = utc_final_nsecs

                            gps_pub.publish(msg)
                        

                        except Exception as e:
                                print("Error Encountered" , e) 
                                msg.header.frame_id = 'GPS1_Frame'
                                msg.altitude = 0.0
                                # utm_coordinate = utm.from_latlon(msg.latitude, msg.longitude)
                                #msg.time= float(gps_puck_data_str[1])
                                msg.utm_easting = 0.0
                                msg.utm_northing = 0.0
                                msg.zone = 0.0
                                msg.letter = 'Error'

                                utc = (gps_puck_data_str[1])
                                hours = int(utc[:2])
                                minutes = int(utc[2:4])
                                seconds = int(utc[4:6])
                                milliseconds = float(utc[6:])
                                
                                utcinSecs = hours * 3600 + minutes * 60 + seconds + milliseconds / 1000.0
                                
                                TimeSinceEpoch = time.time() # Get time since epoch
                                TimeSinceEpochBOD = TimeSinceEpoch - (TimeSinceEpoch % 86400) # Get time since epoch at the beginning of the day (86400 seconds in a day)
                                
                                CurrentTime = TimeSinceEpochBOD + utcinSecs
                                utc_final_secs = int(CurrentTime) # Total seconds
                                utc_final_nsecs = int((CurrentTime - utc_final_secs) * 1e9) # Remaining nanoseconds
                                print(utc_final_secs , utc_final_nsecs)
                                msg.header.stamp.secs = int(utc_final_secs)
                                msg.header.stamp.nsecs = utc_final_nsecs
                                msg.hdop = 0.0
                                msg.fix_quality = 1
                                gps_pub.publish(msg)
                                pass  

                    except: 
                        rospy.logwarn("Exception" + line)
                        continue
    except rospy.ROSInterruptException:
        port.close()
    
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down gps node...")            


