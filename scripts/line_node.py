#!/usr/bin/python
"""
    Controller for omni-robotino for line moving.
"""
import rospy
import threading, sys
from tf.transformations import euler_from_quaternion
from Clock import Clock
from .libs.ControlLaws import LineControlLaw

from geometry_msgs.msg import Twist
from robotino_msgs.msg import NorthStarReadings

lock = threading.Lock()


class Controller:

    def __init__(self, robot_name, v, beta, phi):
        file_name = "data_{}".format(robot_name)
        self.file = open(file_name, 'w')

        sub_topic = "/{}/north_star".format(robot_name)
        pub_topic = "/{}/cmd_vel".format(robot_name)

        self.sub_pose = rospy.Subscriber(sub_topic, NorthStarReadings, self.control_callback)
        self.pub_vels = rospy.Publisher(pub_topic, Twist, queue_size=10)

        # Trajectory setup.
        self.line_law = LineControlLaw(v, beta, phi)
        self.clock = Clock()
        self.work()

    def __del__(self):
        self.file.close()

    def control_callback(self, cur_pose):
        t, dt = self.clock.getTandDT()
        lock.acquire()
        q = cur_pose.pose.orientation
        cur_theta = euler_from_quaternion((q.x, q.y, q.z, q.w))[2]
        ux, uy, e = self.line_law.getControl(cur_pose.pose.position.x, cur_pose.pose.position.y, cur_theta)
        lock.release()

        velocity = Twist()
        velocity.linear.x = ux
        velocity.linear.y = uy
        velocity.angular.z = 2 * (pi / 2 - cur_theta)  # !!! rotation controller
        self.pub_vels.publish(velocity)

        self.file.write("{} {} {} {}\n".format(t, cur_pose.pose.position.x, cur_pose.pose.position.y, e))

    def work(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__=="__main__":
    """
        Read argumets from terminal.
        
        Usege: line_node.py robot_name v beta phi
        beta: angle between Y and line in global frame
        phi: value on the Y-asis
    """
    rospy.init_node('circle_node')

    if len(sys.argv) == 5:
        robot_name = sys.argv[1]
        (v, beta, phi) = sys.argv[2:]

        contrller = Controller(robot_name, v, beta, phi)
        contrller.work()
    else:
        print('Usege: line_node.py robot_name v beta phi')