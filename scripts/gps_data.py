#!/usr/bin/env python

'''gps_data.py is a ROS node that publishes parsed GPS data for use by other ROS nodes 

Date Last Updated: 29/9/19 by Marcel Masque

Purpose: Publish GPS data so that other ROS nodes relying on GPS data can operate correctly

Published topics:
    /gps/gps_data

Data Format:

##Must determine whether to use sensor_msgs/NavSatFix message or create a custom one##
using custom message format for now as sensor_msgs doesnt include time.

GPS data formats obtained from "gpsinformation.org/dale/nmea.htm"
'''

import rospy
import serial
from gps.msg import GPSData
from gps_info import GPSInfo

def gpsData():
	'''function to create a ros node and publish GPS sensor data'''
	pub = rospy.Publisher('/gps/gps_data', GPSData, queue_size=10)
	rospy.init_node('gps_data', anonymous=True)

	# for testing, set an appropriate rate. This wont be needed when there's actual GPS data
	# as the rate of publish will be determined by the GPS data influx rate.
	rate = rospy.Rate(100)
	msg = GPSData()

	#initialise serial to read GPS data from
	ser = serial.Serial()
	try:
		ser.baudrate = 9600
		ser.port = '/dev/serial0'
		#specify timeout so serial doesnt hang
		ser.timeout = 1
		ser.open()
	except:
		pass

	####TESTING####
	data_file = open("sample_data_30092019.txt", "r")
	####END TESTING####

	while not rospy.is_shutdown():
		try:
			data = ser.readline()
			
		except:
			#test that it's working using sample_data
			#####TESTING####
			data = data_file.readline()
			rate.sleep()
			####END TESTING####
		if data[0:6] == "$GPGGA":
			gps_info = GPSInfo()
			gps_info.parseGPS(data)

			msg.latitude = gps_info.latitude
			msg.longitude = gps_info.longitude
			msg.time = gps_info.time
			rospy.loginfo(str(msg))

			pub.publish(msg)




if __name__=="__main__":
    try:
        gpsData()
    except rospy.ROSInterruptException:
        pass
