#!/usr/bin/env python3
"""ROS Noetic node: LaserScan on /scan -> Twist on /cmd_vel.

    rosrun wall_follow_turtlebot wall_follow.py _controller:=v2
"""
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from wall_follower_core import CONTROLLERS


class WallFollowerNode:
    def __init__(self):
        rospy.init_node("wall_follower")
        name = rospy.get_param("~controller", "v1")
        self.controller = CONTROLLERS[name]()
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("scan", LaserScan, self.on_scan, queue_size=1)
        self.last = None
        rospy.on_shutdown(lambda: self.pub.publish(Twist()))
        rospy.loginfo("wall follower running with controller %s", name)

    def on_scan(self, msg: LaserScan):
        now = msg.header.stamp.to_sec() or rospy.get_time()
        dt = 0.0 if self.last is None else now - self.last
        self.last = now
        v, w = self.controller.step(np.asarray(msg.ranges, dtype=float), dt)
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = v, w
        self.pub.publish(cmd)


if __name__ == "__main__":
    WallFollowerNode()
    rospy.spin()
