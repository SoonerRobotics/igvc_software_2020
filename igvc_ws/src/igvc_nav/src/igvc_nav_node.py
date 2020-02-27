#!/usr/bin/env python
import rospy
import math
import tf
from pure_pursuit import PurePursuit
from nav_msgs.msg import Path, Odometry
from igvc_msgs.msg import motors, ekf_state

#temps
import matplotlib as mpl
from matplotlib import pyplot as plt

pos = None
heading = None
publy = rospy.Publisher('/igvc/motors_raw', motors, queue_size=1)

pp = PurePursuit()

def ekf_update(data):
    global pos, heading

    # position = data.pose.pose.position
    # orientation = data.pose.pose.orientation

    # pos = (position.x, position.y)
    # quaternion = (
    #     orientation.x,
    #     orientation.y,
    #     orientation.z,
    #     orientation.w)
    # heading = math.degrees(tf.transformations.euler_from_quaternion(quaternion)[2])

    pos = (data.x_k[3], data.x_k[4])
    heading = 360 - math.degrees(data.x_k[5])

def global_path_update(data):
    points = [x.pose.position for x in data.poses] # Get points from Path
    pp.set_points([(_point.x, _point.y) for _point in points]) # Give PurePursuit the points

def timer_callback(event):
    if pos is None or heading is None:
        return

    lookahead = None
    radius = 0.1 # Start with a radius of 0.1 meters

    while lookahead is None and radius <= 2: # Look until we hit 2 meters max
        lookahead = pp.get_lookahead_point(pos[0], pos[1], radius)
        radius *= 1.25
    
    plt.figure(2)
    plt.clf()
    plt.xlim([-10, 10])
    plt.ylim([-10, 10])
    plt.plot(pos[0], pos[1], 's', markersize=16)

    for point in pp.path:
        plt.plot(point[0], point[1], '.', markersize=8)

    if lookahead is not None:

        plt.plot(lookahead[0], lookahead[1], 'x', markersize=16)

        heading_to_la = math.degrees(math.atan2(pos[1] - lookahead[1], lookahead[0] - pos[0]))
        if heading_to_la < 0:
            heading_to_la += 360

        # print('my x: ' + str(round(pos[0], 2)))
        # print('my y: ' + str(round(pos[1], 2)))

        # print('loc x: ' + str(round(lookahead[0], 2)))
        # print('loc y: ' + str(round(lookahead[1], 2)))

        # print('want to move x:' + str(lookahead[0] - pos[0]))
        # print('want to move y:' + str(lookahead[1] - pos[1]))

        # print('i think my heading is ' + str(heading))
        # print('i want heading ' + str(heading_to_la))
        delta = heading_to_la - heading
        delta = (delta + 180) % 360 - 180

        # print('diff', delta)

        percent_bad = delta/180

        motor_pkt = motors()
        motor_pkt.left = 0.7 * (1 - abs(percent_bad)) + 0.3 * percent_bad
        motor_pkt.right = 0.7 * (1 - abs(percent_bad)) - 0.3 * percent_bad
        
        publy.publish(motor_pkt)

    plt.draw()
    plt.pause(0.00000000001)


def nav():
    rospy.init_node('nav_node', anonymous=True)

    rospy.Subscriber("/igvc_ekf/filter_output", ekf_state, ekf_update)
    rospy.Subscriber("/igvc/global_path", Path, global_path_update)
    
    rospy.Timer(rospy.Duration(0.03), timer_callback)

    plt.ion()
    plt.show()

    rospy.spin()

if __name__ == '__main__':
    try:
        nav()
    except rospy.ROSInterruptException:
        pass

