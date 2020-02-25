#!/usr/bin/env python
import rospy
import math
from pure_pursuit import PurePursuit
from igvc_msgs.msg import EKFState, path, motors

pos = None
heading = None
publy = rospy.Publisher('/igvc/motors_raw', motors, queue_size=1)
pp = PurePursuit()
pp.add_point(-97.4386215209961, 35.1949625611)
pp.add_point(-97.4386215209961, 35.1950436830485)
pp.add_point(-97.4388850511561, 35.1950436830485)
pp.add_point(-97.4388850511561, 35.194881439209)
pp.add_point(-97.4387313252294, 35.194935520488)
pp.add_point(-97.4386215209961, 35.194881439209)
pp.add_point(-97.4386215209961, 35.1949625611)

def ekf_update(data):
    global pos, heading
    pos = (data.x_k[1] * 180/math.pi, data.x_k[0] * 180/math.pi)
    heading = data.x_k[2] * 180/math.pi

def path_update(data):
    global pp
    pp.set_points([(_point.x, _point.y) for _point in data.points])

def timer_callback(event):
    if pos is None or heading is None:
        return
    lookahead = None
    radius = 0.00002
    while lookahead is None:
        lookahead = pp.get_lookahead_point(pos[0], pos[1], radius) # radius about 1 meter
        radius *= 1.5
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

    rospy.Subscriber("/igvc_ekf/filter_output", EKFState, ekf_update)
    rospy.Subscriber("/igvc/path", path, path_update)

    rospy.Timer(rospy.Duration(0.05), timer_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        nav()
    except rospy.ROSInterruptException:
        pass

