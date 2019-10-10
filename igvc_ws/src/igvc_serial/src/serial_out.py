#!/usr/bin/env python

import rospy
import json
import serial
import os
import re
from igvc_msgs.msg import motors

serials = {}

def motors_out(data):
    motion_pkt = {
        "motorLeft": data.left,
        "motorRight": data.right
    }

    json_dump = json.dumps(motion_pkt)

    serials["motor"].write(json_dump)

def serial_out():
    rospy.init_node("serial_out", anonymous = True)
    rospy.Subscriber("/igvc/motors_raw", motors, motors_out)

    # TODO: Create a map from device name (motor) to /dev/ name (/dev/igvc-nucleo-x)
    # TODO: Use map to check if any devices are not plugged in
    serials["motor"] = serial.Serial(port = '/dev/igvc-nucleo-120', baudrate = 9600, timeout = 1)

    rospy.spin()

if __name__ == '__main__':
    try:
        serial_out()
    except rospy.ROSInterruptException:
        pass
