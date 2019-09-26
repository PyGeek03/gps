#!/usr/bin/env python

'''gps_data.py is a ROS node that publishes parsed GPS data for use by other ROS nodes 

Date Last Updated: 26/9/19 by Josh Cherubino

Purpose: Publish GPS data so that other ROS nodes relying on GPS data can operate correctly

Published topics:
    /gps/gps_data

Data Format:
##Must determine whether to use sensor_msgs/NavSatFix message or create a custom one##

GPS data formats obtained from "gpsinformation.org/dale/nmea.htm"
'''

import rospy
#add custom message import statement

def gps_data():
    '''function to create a ros node and publish GPS sensor data'''
    
    pub = rospy.Publisher('/gps/gps_data', GPSData, queue_size=10)
    rospy.init_node('gps_data', anonymous=True)
    #Check required rate for publishing data 
    rate = rospy.Rate(10)
    msg = GPSData()
    while not rospy.is_shutdown():
        #insert message data here
        pub.publish(msg)
        rate.sleep()

if __name__=="__main__":
    try:
        gps_data()
    except rospy.ROSInterruptException:
        pass
