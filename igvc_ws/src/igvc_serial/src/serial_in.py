#!/usr/bin/env python

import rospy
import json
import serial
import os
from std_msgs.msg import String

serials = {}

def serial_out():
    rospy.init_node("serial_out", anonymous = True)
    pubber = rospy.Publisher("/igvc/nucleo_raw", String, queue_size=10)

    # TODO: Create a map from device name (motor) to /dev/ name (/dev/igvc-nucleo-x)
    # TODO: Use map to check if any devices are not plugged in
    serials["motor"] = serial.Serial(port = '/dev/igvc-nucleo-120', baudrate = 9600, timeout = 1)

    while True:
        line = serials[0].readline()
        pubber.publish(line)
            

if __name__ == '__main__':
    try:
        serial_out()
    except rospy.ROSInterruptException:
        pass
