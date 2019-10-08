#!/usr/bin/env python

import rospy
import json
import serial
import os
from igvc_msgs.msg import motors

serials = []

def motors_out(data):
    motion_pkt = {
        "motorLeft": data.left,
        "motorRight": data.right
    }

    json_dump = json.dumps(motion_pkt)

    for s in serials:
        s.write(json_dump)

    rospy.loginfo("Left speed: %f, Right speed: %f, \n", data.left, data.right)

def serial_out():
    rospy.init_node("serial_out", anonymous = True)
    rospy.Subscriber("/igvc/motors_raw", motors, motors_out)

    # TODO: Send hello packet to figure out what device they are
    for i in range(10):
        if os.path.exists('/dev/igvc-nucleo-' + str(i)):
            s = serial.Serial(port = '/dev/igvc-nucleo-' + str(i), baudrate = 9600)
            serials.append(s)

    rospy.loginfo("%d serial devices found!", len(serials))

    if len(serials) > 0:
        rospy.spin()

if __name__ == '__main__':
    try:
        serial_out()
    except rospy.ROSInterruptException:
        pass
