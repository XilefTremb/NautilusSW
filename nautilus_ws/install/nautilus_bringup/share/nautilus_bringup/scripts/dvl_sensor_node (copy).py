#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import String

import time
import serial
import math
# import datetime
# import struct
# import matplotlib.pyplot as plt

AUV_PUBLISHING_FREQUENCY_HZ = 20   # Hz

#ser = serial.Serial(
#        port="/dev/ttyUSB0",
#        baudrate = 115200,
#        parity=serial.PARITY_NONE,
#        stopbits=serial.STOPBITS_ONE,
#        bytesize=serial.EIGHTBITS,
#        timeout=1
#)
#msg_to_dvl = "SEND-GPRMC OFF\r"
#ser.write(msg_to_dvl.encode())

class DVLSensor(Node):
    
    previous_dvl_reading: String
    dvl_publisher: Publisher

    def __init__(self):
        super().__init__("dvl_sensor_node")
        self.previous_dvl_reading = None
        self.dvl_publisher = self.create_publisher(String, "dvl_pub", 1)

        # Timer
        self._timer_period = 1/AUV_PUBLISHING_FREQUENCY_HZ
        self.create_timer(self._timer_period, self.refresh_dvl_read)

    def refresh_dvl_read(self):
        current_dvl_reading = self.read_dvl()
        #if not current_dvl_reading == self.previous_dvl_reading:
        self.dvl_publisher.publish(current_dvl_reading)
        self.previous_dvl_reading = current_dvl_reading

    def read_dvl(self) -> String:
        temp_dvl_reading = String()
        temp_dvl_reading.data = "simulated DVL data"
        time.sleep(0.1) 
        print("Sending")
        return temp_dvl_reading
        
        data_read_by_dvl = ser.readline().decode("utf_8").rstrip('\n\r')
        print(data_read_by_dvl)

        # Format is: $DVEXT,v,g,abcd,r.r,p.p,h.h,k,u.uu,t.tt,n.nnn,e.eee,lat,
        # long,e.eee,qw,qx,qy,qz,ga,gb,gc,gd,la,lb,lc,ld,va,vb,vc,vd,ra,rb,rc,rd,*hh

        data_read_by_dvl_list = data_read_by_dvl.split(",")

        t = self.get_clock().now()

        temp_dvl_reading.header.stamp = t.to_msg()
        if data_read_by_dvl_list[1] == "T":
            temp_dvl_reading.lock = True
        else: 
            temp_dvl_reading.lock = False
        temp_dvl_reading.roll = float(data_read_by_dvl_list[4])
        temp_dvl_reading.pitch = float(data_read_by_dvl_list[5])
        temp_dvl_reading.heading = float(data_read_by_dvl_list[6])
        theta = math.radians(360 - temp_dvl_reading.heading)
        temp_dvl_reading.vx =  float(data_read_by_dvl_list[10])*math.cos(theta) + float(data_read_by_dvl_list[11])*math.sin(theta)
        temp_dvl_reading.vy = -float(data_read_by_dvl_list[10])*math.sin(theta) + float(data_read_by_dvl_list[11])*math.cos(theta)
        temp_dvl_reading.alt = float(data_read_by_dvl_list[9])  # is it really altitude?

        time.sleep(0.1) 
        return temp_dvl_reading

if __name__ == "__main__":
    rclpy.init()
    node = DVLSensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
