#!/usr/bin/env python3
# coding:utf-8

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from loguru import logger


def _laser_scan_callback(msg):
    print(msg,flush=False)

def _sunny_state_callback(msg):
    print(msg.pose.pose.position.x, msg.pose.pose.position.y,msg.pose.pose.orientation.z)

if __name__=="__main__":

    # laser_scan = None
    rospy.init_node("ros_rl", anonymous=True)#初始化节点 名称：test

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

