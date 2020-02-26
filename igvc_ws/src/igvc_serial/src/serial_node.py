#!/usr/bin/env python

import rospy
import json
import threading
import serial

from std_msgs.msg import String
from igvc_msgs.msg import motors, velocity, gps

# ROS node that facilitates all serial communications within the robot
# Subscribes to motor values
# Publishes GPS coordinates

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

# Constructs motor message from given data and sends to serial
def motors_out(data):

    # JSON packet to be sent
    motion_pkt = {
        "motorLeft": data.left,
        "motorRight": data.right,
    }

    # Encode JSON string to bytes
    json_dump = json.dumps(motion_pkt, separators = (',', ':'))
    out = (json_dump + "\n").encode()

    serials["motor"].write(out)

# Receives longitude and latitude coordinates from the GPS,
# then publishes them as a gps message
def get_gps_sensor_data(timer_event):
    
    global gps_serial
    global gps_pub

    # Read the coordinate packet
    coord = gps_serial.read_until()

    # Construct gps message with given packet and publish
    try:
        coord_json = json.loads(coord)

        coord_msg = gps()
        coord_msg.latitude = coord_json['latitude']
        coord_msg.longitude = coord_json['longitude']
        coord_msg.hasSignal = coord_json['hasSignal']

        gps_pub.publish(coord_msg)

    except ValueError:
        pass


# Initialize the serial node
# Node handles all serial communication within the robot (motor, GPS)
def init_serial_node():

    global gps_serial
    global gps_pub

    # Setup serial node
    rospy.init_node("serial_node", anonymous = False)

    # Setup motor serial and subscriber
    motor_serial = serials["motor"] = serial.Serial(port = '/dev/igvc-nucleo-120', baudrate = 115200)
    motor_sub = rospy.Subscriber('/igvc/motors_raw', motors, motors_out)
    
    # # The following lines are an example of how to create a serial reading object thread
    motor_response_thread = SerialReadThread(serial_obj = serials["motor"], topic = '/igvc/velocity')
    motor_response_thread.start()

    # Setup GPS serial and publisher
    gps_serial = serials["gps"] = serial.Serial(port = '/dev/ttyACM0', baudrate = 9600)
    gps_pub = rospy.Publisher('/igvc/gps', gps, queue_size = 10)
    gps_sensor_timer = rospy.Timer(rospy.Duration(secs = 0.5), get_gps_sensor_data)
    
    # Wait for topic updates
    rospy.spin()

    # Close the serial ports when program ends
    motor_serial.close()
    gps_serial.close()

if __name__ == '__main__':
    try:
        init_serial_node()
    except rospy.ROSInterruptException:
        pass
