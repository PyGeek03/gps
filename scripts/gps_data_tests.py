"""
Testing parsing functionality for the GPSData class in gps_data
Last modified on 27/9/19 by Marcel Masque

"""



from gps_data import GPSInfo

gps = GPSInfo()
gps.parseGPS("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47")
assert gps.latitude -  48.1173 < 0.000000001,  "Latitude parsing failed"
assert gps.longitude - 11.5166666667 < 0.000000001, "Longitude parsing failed"



