#!/usr/bin/env python
import rospy
import math
from pure_pursuit import PurePursuit
from igvc_msgs.msg import ekf_state, gps, path, motors

pos = None
heading = None
pp = PurePursuit()

def ekf_update(data):
    pos = (data.x_k[1], data.x_k(2))
    heading = data.x_k[3]

def path_update(data):
    pp.set_points(data.points)

def nav():
    rospy.init_node('nav_node', anonymous=True)

    rospy.Subscriber("/igvc_ekf/filter_output", ekf_state, ekf_update)
    rospy.Subscriber("/igvc/path", path, path_update)

    publy = rospy.Publisher('/igvc/motors', motors, 1)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        lookahead = pp.get_lookahead_point(pos[0], pos[1], 0.00001) # radius about 1 meter
        if lookahead is not None:
            heading_to_la = 90 - math.atan2(lookahead[1] - pos[1], lookahead[0] - pos[0]) * 180 / (math.pi)
            if heading_to_la < 0:
                heading_to_la += 360

            delta = heading_to_la - heading
            delta = (a + 180) % 360 - 180

            motor_pkt = motors()
            motor_pkt.left = 0.2 + 0.8(delta / 180)
            motor_pkt.right = 0.2 + 0.8(delta / 180)
            
            publy.publish(motor_pkt)
        rate.sleep()

if __name__ == '__main__':
    try:
        nav()
    except rospy.ROSInterruptException:
        pass

