#!/usr/bin/env python
import rospy
from matplotlib import pyplot as plot

def display_map_callback(data):
    plot.plot(data)
    plot.show
    print(matplotlib.get_backend())

def map():
    rospy.init_node('map_node', anonymous=True)

    rospy.Subscriber("/igvc_vision/map", OccupancyGrid, display_map_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        map()
    except rospy.ROSInterruptException:
        pass

