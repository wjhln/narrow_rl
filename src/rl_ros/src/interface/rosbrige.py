#!/usr/bin/env python
# coding:utf-8

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Polygon, Point32
from geometry_msgs.msg import PolygonStamped
import tf
import math
import numpy as np


def _laser_scan_callback(msg):
    print(msg)

def _sunny_state_callback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    euler = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    yaw = euler[2]
    print(x, y, yaw)

  
    car_RB = 0.25
    car_RF = 0.62
    car_W = 0.76
    car = np.array([[-car_RB, -car_RB, car_RF, car_RF],
                    [car_W / 2, -car_W / 2, -car_W / 2, car_W / 2]])
    rot1 = np.array([[math.cos(yaw), -math.sin(yaw)],
                     [math.sin(yaw), math.cos(yaw)]])
    car = np.dot(rot1, car)
    car += np.array([[x], [y]])

    polygon_msg = PolygonStamped()
    polygon_msg.header.frame_id = 'base_link'
    polygon_msg.polygon.points = [
        Point32(x=car[0][0], y=car[1][0], z=0.0),
        Point32(x=car[0][1], y=car[1][1], z=0.0),
        Point32(x=car[0][2], y=car[1][2], z=0.0),
        Point32(x=car[0][3], y=car[1][3], z=0.0)
    ]

    polygon_pub_.publish(polygon_msg)


if __name__=="__main__":

    # laser_scan = None
    rospy.init_node("ros_rl", anonymous=True)#初始化节点 名称：test
    polygon_pub_ = rospy.Publisher('/polygon', PolygonStamped, queue_size=10)

    # logger.info("Waiting for /scan to be READY...")
    # rospy.Subscriber("/scan", Twist, _laser_scan_callback)
    # while laser_scan is None and not rospy.is_shutdown():
    #     try:
    # laser_scan = rospy.wait_for_message("/sunny/state", Twist, timeout=5.0)
    #         rospy.logdebug("Current /scan READY=>")

    #     except:
    #         rospy.logerr("Current /scan not ready yet, retrying for getting laser_scan")
    # print(laser_scan)
    # rospy.Subscriber("/scan", LaserScan, _laser_scan_callback)
    rospy.Subscriber("/sunny/state", Odometry, _sunny_state_callback)

    rospy.spin()

