#!/usr/bin/python
"""
    Controller for omni-robotino for circle moving.
"""
import rospy, threading, sys
from numpy import pi

from .line_node import Controller as LineCntrl
from .circle_node import Controller as CircleCntrl
from .sin_node import Controller as SinCntrl

lock = threading.Lock()


if __name__ == "__main__":
    """
    Work of this script are 
        'robot1' following given trajectory,
        'robot2' moving around 'robot1' for circle trajectory.
    """
    rospy.init_node('composite_node')

    if len(sys.argv) == 3:
        robot_name = sys.argv[1]
        (x0, y0) = sys.argv[2:]

        some_curve = LineCntrl('robot1', 0.05, pi/4, 0)
        # some_curve = SinCntrl('robot1', 0.05, 1)
        # some_curve = CircleCntrl('robot1', 0.05, 0.8)
        circle_controller = CircleCntrl('robot2', 0.5, 0.8, center=(float(x0), float(y0)))

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            lock.acquire()
            (x_r1, y_r1, __) = some_curve.state
            circle_controller.set_center((x_r1, y_r1))
            lock.release()
            rate.sleep()
    else:
        print("Usage: composite_node.py x y \n (x,y) coordinates of 'robot1'.")
