#!/usr/bin/env python

import rospy
import json
import threading
import serial
from std_msgs.msg import String
from igvc_msgs.msg import motors
from igvc_msgs.msg import velocity

serials = {}

class SerialReadThread(threading.Thread):
    def __init__(self, serial_obj, topic):
        threading.Thread.__init__(self)

        self.serial_obj = serial_obj

        # Allow timeout of up to 1 second on reads. This could be set to None to have infinite timeout,
        # but that would hault the node when it tries to exit. Need to make sure the while loop condition is
        # semi-regularly checked. This is better than rospy.Rate because it will continously wait for new message
        # instead of only checking on a fixed interval.
        self.serial_obj.timeout = 1

        # Assumes String type for now. This class will need to be adapted in the future for different message types.
        self.publisher = rospy.Publisher(topic, velocity, queue_size=1)

    def run(self):
        while not rospy.is_shutdown():
            line = self.serial_obj.readline() # Assume all messages end in newline character. This is standard among SCR IGVC serial messages.

            if line:
                print(line)
                vels = line.split(",")
                velPkt = velocity()
                velPkt.leftVel = float(vels[0])
                velPkt.rightVel = float(vels[1])

                self.publisher.publish(velPkt)


def motors_out(data):
    motion_pkt = {
        "motorLeft": data.left,
        "motorRight": data.right,
    }

    json_dump = json.dumps(motion_pkt, separators=(',', ':'))
    # print("writing " + json_dump)
    out = (json_dump + "\n").encode() # encode json string to bytes

    serials["motor"].write(out)

def serial_node():

    # Setup node
    rospy.init_node("serial_node", anonymous = True)

    # Create Serial objects to read and write from serial devices
    serials["motor"] = serial.Serial(port = '/dev/igvc-nucleo-120', baudrate = 115200)

    # Subscribe to necessary topics
    rospy.Subscriber("/igvc/motors_raw", motors, motors_out)
    
    # # The following lines are an example of how to create a serial reading object thread
    motor_response_thread = SerialReadThread(serial_obj = serials["motor"], topic = '/igvc/velocity')
    motor_response_thread.start()

    # Wait for topic updates
    rospy.spin()

if __name__ == '__main__':
    try:
        serial_node()
    except rospy.ROSInterruptException:
        pass
