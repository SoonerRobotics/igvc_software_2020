#!/usr/bin/env python
import rospy
import math
import tf
from pure_pursuit import PurePursuit
from nav_msgs.msg import Path, Odometry
from igvc_msgs.msg import motors

pos = None
heading = None
publy = rospy.Publisher('/igvc/motors_raw', motors, queue_size=1)

pp = PurePursuit()

def odom_update(data):
    global pos, heading

    position = data.pose.pose.position
    orientation = data.pose.pose.orientation

    # pos = (position.x, position.y)
    # quaternion = (
    #     orientation.x,
    #     orientation.y,
    #     orientation.z,
    #     orientation.w)
    # heading = math.degrees(tf.transformations.euler_from_quaternion(quaternion)[2])
    
    
    # lets just ignore odom because we arent even global LOL
    pos = (100,100)
    heading = 0

def local_path_update(data):
    points = [x.pose.position for x in data.poses] # Get points from Path
    pp.set_points([(_point.x, _point.y) for _point in points]) # Give PurePursuit the points

def timer_callback(event):
    if pos is None or heading is None:
        return

    lookahead = None
    radius = 0.3 # Start with a radius of 0.1 meters

    while lookahead is None and radius <= 2: # Look until we hit 2 meters max
        lookahead = pp.get_lookahead_point(pos[0], pos[1], radius)
        radius *= 1.25
    
    if lookahead is not None:
        heading_to_la = 90 - math.atan2(lookahead[1] - pos[1], lookahead[0] - pos[0]) * 180 / (math.pi)
        if heading_to_la < 0:
            heading_to_la += 360

        # print('my x: ' + str(pos[0]))
        # print('my y: ' + str(pos[1]))

        # print('loc x: ' + str(lookahead[0]))
        # print('loc y: ' + str(lookahead[1]))

        # print('want to move x:' + str(lookahead[0] - pos[0]))
        # print('want to move y:' + str(lookahead[1] - pos[1]))

        # print('i think my heading is ' + str(heading))
        # print('i want heading ' + str(heading_to_la))
        delta = heading_to_la - heading
        delta = (delta + 180) % 360 - 180

        motor_pkt = motors()
        motor_pkt.left = 0.4 + 0.6 * (delta / 180)
        motor_pkt.right = 0.4 - 0.6 * (delta / 180)
        
        publy.publish(motor_pkt)

def nav():
    rospy.init_node('nav_node', anonymous=True)

    rospy.Subscriber("/odom", Odometry, odom_update)
    rospy.Subscriber("/igvc/local_path", Path, local_path_update)
    
    rospy.Timer(rospy.Duration(0.05), timer_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        nav()
    except rospy.ROSInterruptException:
        pass

