#!/usr/bin/env python

import rospy
import json
import threading
import serial

from std_msgs.msg import String
from igvc_msgs.msg import motors, gps

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
        self.publisher = rospy.Publisher(topic, String, queue_size=10)

    def run(self):
        while not rospy.is_shutdown():
            line = self.serial_obj.readline() # Assume all messages end in newline character. This is standard among SCR IGVC serial messages.

            if line:
                self.publisher.publish(line)

def motors_out(data):
    motion_pkt = {
        "motorLeft": data.left,
        "motorRight": data.right
    }

    json_dump = json.dumps(motion_pkt, separators = (',', ':'))
    out = (json_dump + "\n").encode() # encode json string to bytes

    serials["motor"].write(out)

def get_gps_sensor_data(timer_event):
    global gps_serial
    global gps_pub

    # Read the coordinate packet
    coord = gps_serial.read_until()

    # Create gps message from json packet read
    try:
        coord_json = json.loads(coord)
        coord_msg = gps()

        coord_msg.latitude = coord_json['latitude']
        coord_msg.longitude = coord_json['longitude']

        gps_pub.publish(coord_msg)

    except ValueError:
        pass


def init_serial_node():

    # Setup node
    rospy.init_node("serial_node", anonymous = True)

    # Create Serial objects to read and write from serial devices
    motor_serial = serials["motor"] = serial.Serial(port = '/dev/igvc-nucleo-319', baudrate = 115200)
    gps_serial = serials["gps"] = serial.Serial(port = '', baudrate = 115200)

    # Set up a publisher for publishing gps coordinates
    gps_pub = rospy.Publisher('/igvc/gps', gps, queue_size = 10)
    # Reads gps coordinates at 200 Hz
    gps_sensor_timer = rospy.Timer(rospy.Duration(secs = 0.005), get_gps_sensor_data)

    # Subscribe to necessary topics
    motors_sub = rospy.Subscriber('/igvc/motors_raw', motors, motors_out)
    
    # Wait for topic updates
    rospy.spin()

    # Close the serial ports when program ends
    motor_serial.close()
    gps_serial.close()

    # # The following lines are an example of how to create a serial reading object thread
    # motor_response_thread = SerialReadThread(serial_obj = serials["motor"], topic = '/igvc/serial/motors_in')
    # motor_response_thread.start()


if __name__ == '__main__':
    try:
        init_serial_node()
    except rospy.ROSInterruptException:
        pass
