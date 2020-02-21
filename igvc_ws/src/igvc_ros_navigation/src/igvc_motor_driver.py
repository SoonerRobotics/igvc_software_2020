#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist
from igvc_msgs.msg import motors

motors_pub = rospy.Publisher("/igvc/motors_raw", motors, queue_size=1)

def cmd_callback(cmd):
    """ Handle velocity commands from the navigation controllers """
    left_motor = cmd.linear.x - cmd.angular.z
    right_motor = cmd.linear.x + cmd.angular.z
    motors_cmd = motors(left = left_motor, right = right_motor)

    motors_pub.publish(motors_cmd)


def igvc_motor_driver():
    """ Make a translator between the navigation stack and the robot """
    # Set up node
    rospy.init_node("igvc_motor_driver")

    # Subscribe to necessary topics
    map_sub = rospy.Subscriber("/cmd_vel", Twist, cmd_callback, queue_size=10)

    # Wait for topic updates
    rospy.spin()


# Main setup
if __name__ == '__main__':
    try:
        igvc_motor_driver()
    except rospy.ROSInterruptException:
        pass