#!/usr/bin/env python
import socket
import rospy
from sensor_msgs.msg import NavSatFix

class GPS_Parser(object):
    
    def __init__(self):
        rospy.init_node('gps_parser')
        
        self.fix_pub = rospy.Publisher('fix', NavSatFix, queue_size = 10)
        
        self.UDP_IP = ""
        self.UDP_PORT = 8308
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.UDP_IP, self.UDP_PORT))
        
        rospy.sleep(1)
        
        while not rospy.is_shutdown():
            self.get_and_pub_packet()
        
    def get_and_pub_packet(self):
        data, addr = self.sock.recvfrom(8308)
        line = data[206:278]
  
        my_fix = NavSatFix()
        my_fix.header.stamp = rospy.Time.now()

        latitude_degrees = float(line[16:18])
        latitude_minutes = float(line[18:23])
        latitude = decimal_degrees(degs=latitude_degrees, mins=latitude_minutes)
        if (line[24] == 'S'):
            latitude = -1 * latitude

        longitude_degrees = float(line[26:29])
        longitude_minutes = float(line[29:34])
        longitude = decimal_degrees(degs=longitude_degrees, mins=longitude_minutes)
        if(line[35] == 'W'):
            longitude = -1 * longitude

        my_fix.latitude = latitude
        my_fix.longitude = longitude
        my_fix.altitude = 0.0

        
        
        self.fix_pub.publish(my_fix)

    def decimal_degrees(self, degs=0, mins=0, secs=0):
        return degs + (mins/60.0) + (secs/3600.0)

if __name__ == "__main__":
    GPS_Parser()
