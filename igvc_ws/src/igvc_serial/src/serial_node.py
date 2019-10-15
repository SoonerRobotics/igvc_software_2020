#!/usr/bin/env python

import rospy
import json
import threading
import serial
from std_msgs.msg import String
from igvc_msgs.msg import motors

serials = {}

class SerialReadThread(threading.Thread):
    def __init__(self, serial_obj, topic):
        threading.Thread.__init__(self)

        self.serial_obj = serial_obj
        self.publisher = rospy.Publisher(topic, String, queue_size=10)

    def run(self):
        # TODO: Is it good to use rospy.Rate here? Or should we just add a timeout
        # to the Serial object and wait on that?
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            line = self.serial_obj.readline()

            if line:
                self.publisher.publish(line)

            r.sleep()


def motors_out(data):
    motion_pkt = {
        "motorLeft": data.left,
        "motorRight": data.right
    }

    json_dump = json.dumps(motion_pkt, separators=(',', ':'))
    out = (json_dump + "\n").encode() # encode json string to bytes

    serials["motor"].write(out)

def serial_node():

    # Setup node
    rospy.init_node("serial_node", anonymous = True)

    # Create Serial objects to read and write from serial devices
    serials["motor"] = serial.Serial(port = '/dev/igvc-nucleo-319', baudrate = 115200)

    # Subscribe to necessary topics
    rospy.Subscriber("/igvc/motors_raw", motors, motors_out)
    
    # # The following lines are an example of how to create a serial reading object thread
    # motor_response_thread = SerialReadThread(serial_obj = serials["motor"], topic = '/igvc/serial/motors_in')
    # motor_response_thread.start()

    # Wait for topic updates
    rospy.spin()

if __name__ == '__main__':
    try:
        serial_node()
    except rospy.ROSInterruptException:
        pass
