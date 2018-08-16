#!/usr/bin/python
"""
    Controller for omni-robotino for circle moving.
"""
import rospy
import threading, sys
from tf.transformations import euler_from_quaternion
from Clock import Clock
from ControlLaws import CircleControlLaw

from geometry_msgs.msg import Twist
from robotino_msgs.msg import NorthStarReadings

lock = threading.Lock()


class Controller:

    def __init__(self, robot_name, v, R, center=None):
        file_name = "data_{}.txt".format(robot_name)
        self.file = open(file_name, 'w')

        sub_topic = "/{}/north_star".format(robot_name)
        pub_topic = "/{}/cmd_vel".format(robot_name)

        self.sub_pose = rospy.Subscriber(sub_topic, NorthStarReadings, self.control_callback)
        self.pub_vels = rospy.Publisher(pub_topic, Twist, queue_size=10)

        # Trajectory setup.
        self.circle_law = CircleControlLaw(v, R)
        if center:
            self.center = center
        else:
            self.center = (0, 0)

        self.clock = Clock()
        self.state = (0, 0, 0)
        # self.work()

    def __del__(self):
        self.file.close()

    def set_center(self, center):
        self.center = center

    def control_callback(self, cur_pose):
        t, dt = self.clock.getTandDT()
        lock.acquire()
        p = cur_pose.pose.position
        q = cur_pose.pose.orientation
        cur_theta = euler_from_quaternion((q.x, q.y, q.z, q.w))[2]
        ux, uy, e = self.circle_law.getControl(cur_pose.pose.position.x, cur_pose.pose.position.y,
                                               cur_theta, (self.center[0], self.center[1]))
        self.state = (p.x, p.y, cur_theta)
        lock.release()

        velocity = Twist()
        velocity.linear.x = ux
        velocity.linear.y = uy
        self.pub_vels.publish(velocity)

        self.file.write("{} {} {} {}\n".format(t, cur_pose.pose.position.x, cur_pose.pose.position.y, e))

    def work(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__=="__main__":
    """
        Read argumets from terminal.
        
        Usage: circle_node.py robot_name v R x0 y0
    """
    rospy.init_node('circle_node')

    if len(sys.argv) == 4:
        robot_name = sys.argv[1]
        (v, R) = sys.argv[2:]

        contrller = Controller(robot_name, float(v), float(R))
        contrller.work()
    elif len(sys.argv) == 6:
        robot_name = sys.argv[1]
        (v, R, x0, y0) = sys.argv[2:]

        contrller = Controller(robot_name, float(v), float(R), (float(x0), float(y0)))
        contrller.work()
    else:
        print('Usage: circle_node.py robot_name v R x0 y0')
