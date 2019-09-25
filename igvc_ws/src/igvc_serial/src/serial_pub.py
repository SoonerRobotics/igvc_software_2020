#!/usr/bin/env python

import rospy
from igvc_msgs.msg import serial_motion_msg
import serial
import json

def talker():  
    pub = rospy.Publisher('/igvc/motion_serial', serial_motion_msg, queue_size=10)
    rospy.init_node('serial_publisher_node', anonymous = True)

    leftF = 0
    rightF = 0

    rate = rospy.Rate(10)

    s = serial.Serial(port = '/dev/ttyACM0', baudrate = 9600)

    while not rospy.is_shutdown():
        test = serial_motion_msg()
        test.left_speed = leftF
        test.right_speed = rightF
        leftF += 0.0001
        rightF += 0.0001
        pub.publish(test)

        motion_pkt = {
            "motorLeft": leftF,
            "motorRight": rightF
        }
        
        json_msg = json.dumps(motion_pkt)
        print(json_msg)
        
        s.write(json_msg)
        
        rate.sleep()
    
    s.close()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass