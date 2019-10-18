import rospy
from matplotlib import pyplot as plot

def map():
    rospy.init_node('map_node', anonymous=True)

    rospy.Subscriber("/igvc_vision/map", Image, map_callback) # Don't know what Image is or should be
    rospy.spin()

if __name__ == '__main__':
    try:
        map()
    except rospy.ROSInterruptException:
        pass

