#!/usr/bin/env python
"""GPSInfo is a class to parse and store serial inputs from GPS sensors.

Date Last Updated: 30/9/19 by Marcel Masque

Purpose: Decode, parse, compute checksum and store GPS data in a class. 


"""
import rospy
class GPSInfo:
    def __init__(self):
        """Initialise to default values"""
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        #initialise time to 0 seconds, 0 nseconds
        self.time = rospy.Time()

    def parseGPS(self, data):
        """parse GPS data string and populate instance variables
        
        Argument:
        data {str} -- String containing comma separated GPS data

        """
        if data[0:6] == "$GPGGA":
            s =  data.split(",")
            checksum_result = self.computeChecksum(data)
            if not checksum_result:
                return

            if s[7] ==  '0':
                return
            try:
                lat = self.decode(s[2])
                #multiply by 1 or -1 depending on N or S 
                lat_dirs = {"N":1, "S":-1}
                lat_mult = lat_dirs[s[3]]
                self.latitude =lat*lat_mult

                lon = self.decode(s[4]) 
                #multiply by 1 or -1 depending on E or W
                lon_dirs = {"E":1, "W":-1}
                lon_mult = lon_dirs[s[5]]
                self.longitude =lon*lon_mult
            except:
                pass
            
            if s[1][0:2] != "":
                time_secs = 60*60*float(s[1][0:2]) + 60*float(s[1][2:4])  + float(s[1][4:6])
                self.time = rospy.Time(secs=time_secs) 
                
                

    def decode(self, coord):
        """decode a string coordinate in decimal degrees minutes seconds

        Argument:
        coord {str} -- A partial coordinate (latitude or longitude) in decimal degrees min seconds

        Returns: 
        {float} -- A partial coordinate in decimal degrees minutes
        """
        v = coord.split(".")
        head = v[0]
        tail = v[1]
        deg = head[0:-2]
        min_ = head[-2:]
        return float(deg) + float(min_ + "." + tail)/60

    def computeChecksum(self,data):
        """compute a char wise XOR checksum of the data and compare it to the hex value after the * in the data string
        
        Argument:
        data {str} -- String containing comma separated GPS data with checksum result directly after *

        Returns: 
        {bool} -- True if checksum is correct, False otherwise
        """
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
        try:
            checksum_str = "0x" + data.split(',')[-1][1:]
        except:
            return False
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