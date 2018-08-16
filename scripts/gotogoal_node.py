#!/usr/bin/python
"""
    Script for moving omni-robot to the certain goal.
    North-star was used in feedback.
    !!! Only for x and y coordinates; 'theta' still don't used :(
"""
import rospy, sys
import threading
from numpy import sqrt, sin, cos, sign, pi
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from Clock import Clock

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist

from geometry_msgs.msg import Quaternion
from robotino_msgs.msg import NorthStarReadings
from tf.msg import tfMessage

from controlling_omni_robots.srv import GoToGoal

lock = threading.Lock()


class Controller:

    k = 0.5
    k2 = 2

    def __init__(self, robot_name, goal):
        file_name = "data_{}.txt".format(robot_name)
        self.file = open(file_name, 'w')

        sub_topic = "/{}/north_star".format(robot_name)
        pub_topic = "/{}/cmd_vel".format(robot_name)
        srv_topic = "/{}/gotogoal".format(robot_name)

        self.sub_pose = rospy.Subscriber(sub_topic, NorthStarReadings, self.point_callback)
        self.pub_vels = rospy.Publisher(pub_topic, Twist, queue_size=10)
        self.go_to_goal = rospy.Service(srv_topic, GoToGoal, self.go_to_goal_handler)

        self.goal = goal
        self.clock = Clock()
        self.work()

    def __del__(self):
        self.file.close()

    def go_to_goal_handler(self, req):
        self.goal.x = req.point.x
        self.goal.y = req.point.y
        print("New goal is [{}, {}]".format(self.goal.x, self.goal.y))

    def point_callback(self, cur_pose):
        t, dt = self.clock.getTandDT()

        q = cur_pose.pose.orientation
        eu = euler_from_quaternion((q.x, q.y, q.z, q.w))

        lock.acquire()
        e_x = self.goal.x - cur_pose.pose.position.x
        e_y = self.goal.y - cur_pose.pose.position.y
        e_theta = 0 - eu[2]  # self.goal.z - eu[2]
        lock.release()

        e = sqrt(e_x ** 2 + e_y ** 2)  # euclid error

        # self.file.write("{} {} {} {}\n".format(t, cur_pose.pose.position.x, cur_pose.pose.position.y, e))

        velocity = Twist()
        if abs(e) > 0.05:
            # velocity.linear.x = self.k * e_y
            # velocity.linear.y = -self.k * e_x
            velocity.linear.x = self.k * e_x
            velocity.linear.y = self.k * e_y
        velocity.angular.z = 4 * e_theta
        self.pub_vels.publish(velocity)

    def work(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == "__main__":
    """
        Read argumets from terminal.
        usage: gotogoal.py robot_name x y theta
    """
    if len(sys.argv) == 5:
        robot_name = sys.argv[1]
        (x, y, theta) = sys.argv[2:]

        rospy.init_node('the_robot_go_to_goal')
        contrller = Controller(robot_name, goal=Point(float(x), float(y), float(theta)))
        contrller.work()
    else:
        print('Usege: gotogoal.py robot_name x y theta')

