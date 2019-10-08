#!/usr/bin/env python

import rospy
import json
import serial
import os
from std_msgs.msg import String

serials = []

def serial_out():
    rospy.init_node("serial_out", anonymous = True)
    pubber = rospy.Publisher("/igvc/nucleo_raw", String, queue_size=10)

    # TODO: Send hello packet to figure out what device they are
    for i in range(10):
        if os.path.exists('/dev/igvc-nucleo-' + str(i)):
            s = serial.Serial(port = '/dev/igvc-nucleo-' + str(i), baudrate = 9600)
            serials.append(s)

    rospy.loginfo("%d serial devices found!", len(serials))

    if len(serials) > 0:
        while True:
            line = serials[0].readline()
            pubber.publish(line)
            # data = json.loads(line)
            # if 'leftMotor' in data and 'rightMotor' in data:
            #     msg = motors()
            #     msg.left = data['leftMotor']
            #     msg.right = data['rightMotor']
            #     rospy.publish(msg)
            

if __name__ == '__main__':
    try:
        serial_out()
    except rospy.ROSInterruptException:
        pass
