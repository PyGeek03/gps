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
import time
#add custom message import statement
from gps.msg import GPSData

class GPSInfo:
	def	__init__(self):
		self.latitude = None
		self.longitude = None
		self.altitude = None
		self.seconds_since_epoch = time.mktime(time.localtime())
		self.time = rospy.Time.from_sec(self.seconds_since_epoch)

	def parseGPS(self, data):
		if data[0:6] == "$GPGGA":
			s =  data.split(",")
			checksum_result = self.compute_checksum(data)
			if not checksum_result:
				return

			if s[7] ==  '0':
				return
			
			
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
			if s[1][0:2] != "":
				time_str = s[1][0:2] + ":" + s[1][2:4] + ":" + s[1][4:6]
				self.seconds_since_epoch = time.mktime(time.strptime(time_str, "%H:%M:%S"))
				self.time = rospy.Time.from_sec(self.seconds_since_epoch) 
				
				

	def decode(self, coord):
		v = coord.split(".")
		head = v[0]
		tail = v[1]
		deg = head[0:-2]
		min_ = head[-2:]
		return float(deg) + float(min_ + "." + tail)/60

	def compute_checksum(self,data):
		"""compute a char wise XOR checksum of the data and compare it to the hex value after the * in the data string"""
		try:
			#take a substring between $ and *
			s1 = data.split('$')[1]
			s1 = s1.split('*')[0]
		except:
			#if we can't find a $ or a * the data is corrupt; return false
			return False

		#compute char wise checksum
		checksum = 0
		for char in s1:
			checksum ^= ord(char)
		#convert to hex for comparison with checksum value in str
		checksum = hex(checksum)
		#split the data string and access the checksum hex 
		#[1:] to skip over *
		checksum_str = "0x" + data.split(',')[14][1:]
		checksum_int = int(checksum_str, 16)
		hex_checksum = hex(checksum_int)
		if checksum != hex_checksum:
			return False
		else:
			return True

		

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
		ser.port = '/dev/serial0'
		#specify timeout so serial doesnt hang
		ser.timeout = 1
		ser.open()
	except:
		pass

	while not rospy.is_shutdown():
		
		data = ser.readline()
		if data[0:6] == "$GPGGA":
			gps_info = GPSInfo()
			gps_info.parseGPS(data)

			msg.latitude = gps_info.latitude
			msg.longitude = gps_info.longitude
			msg.time = gps_info.time
			rospy
			rospy.loginfo(str(msg))

			pub.publish(msg)



if __name__=="__main__":
    try:
        gps_data()
    except rospy.ROSInterruptException:
        pass
