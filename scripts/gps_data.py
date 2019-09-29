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
from datetime import datetime
#add custom message import statement
from gps.msg import GPSData

class GPSInfo:
	def	__init__(self):
		self.latitude = None
		self.longitude = None
		self.altitude = None
		self.time = None
		
	def parseGPS(self, data):
		if data[0:6] == "$GPGGA":
			s =  data.split(",")
			if s[7] ==  '0':
				print "no satellite data available"
				return
			time_str = s[1][0:2] + ":" + s[1][2:4] + ":" + s[1][4:6]
			
			try:
				lat = self.decode(s[2])
				
				#multiply by 1 or -1 depending on N or S 
				lat_dirs = {"N":1, "S":-1}
				lat_mult = lat_dirs[s[3]]
				lat *= lat_mult
			except:
				lat = None
			try:
				lon = self.decode(s[4])	
				#multiply by 1 or -1 depending on E or W
				lon_dirs = {"E":1, "W":-1}
				lon_mult = lon_dirs[s[5]]
				lon *= lon_mult
				
			except:
				lon = None
			try: 
				alt = s[9] + " m"
				sat = s[7]
			except:
				alt = None
				sat = None
			
			self.latitude = lat
			self.longitude = lon
			try:
				self.time = datetime.strptime(time_str, "%H:%M:%S")
			except:
				pass
		else:
			print 'no run : ( '

	def decode(self, coord):
		v = coord.split(".")
		head = v[0]
		tail = v[1]
		deg = head[0:-2]
		min_ = head[-2:]
		return float(deg) + float(min_ + "." + tail)/60

	def __str__(self):
		retval = "Latitude: " + str(self.latitude) + ", "
		retval += "Longitude: " + str(self.longitude) + ", "
		retval += "Altitude: " + str(self.altitude) + ", "
		retval += "Time(UTC): " + str(self.time) + "\n"
		return retval


			
	


		
		

def gps_data():
	'''function to create a ros node and publish GPS sensor data'''
	pub = rospy.Publisher('/gps/gps_data', GPSData, queue_size=10)
	rospy.init_node('gps_data', anonymous=True)

	# for testing, set an appropriate rate. This wont be needed when there's actual GPS data
	# as the rate of publish will be determined by the GPS data influx rate.
	rate = rospy.Rate(1)
	msg = GPSData()

	#initialise serial to read GPS data from
	ser = serial.Serial()
	try:
		ser.baudrate = 9600
		ser.port = '/dev/ttyS0'
		#specify timeout so serial doesnt hang
		ser.timeout = 1
		ser.open()
	except:
		pass

	while not rospy.is_shutdown():
		try:
			data = ser.readline()
		except: 
			#####TESTING####
			#for testing, fill with good data to test publisher node
			data = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"
			rate.sleep()
			#####END TESTING####
		gps_info = GPSInfo()
		gps_info.parseGPS(data)

		msg.latitude = gps_info.latitude
		msg.longitude = gps_info.longitude
		msg.time = gps_info.time

		rospy.loginfo(str(msg))

		pub.publish(msg)



if __name__=="__main__":
    try:
        gps_data()
    except rospy.ROSInterruptException:
        pass
