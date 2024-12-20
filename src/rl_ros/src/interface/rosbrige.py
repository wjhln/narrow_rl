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
from shapely.geometry import Point, Polygon
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool

def _top_laser_scan_callback(msg):
    # 车框
    yaw = -math.pi / 2
    car_RB = 0.25
    car_RF = 0.62
    car_W = 0.72
    # car = np.array([[car_W / 2, -car_W / 2, -car_W / 2, car_W / 2],
    #                 [-car_RB, -car_RB, car_RF, car_RF]])
    poly_x = [0.61, 0.57, -0.158, -0.208, -0.208, -0.158, 0.57, 0.61]
    poly_y = [-0.29, -0.36, -0.36, -0.278, 0.278, 0.36, 0.36, 0.29]
    car = np.array([poly_x, poly_y])
    rot1 = np.array([[math.cos(yaw), -math.sin(yaw)],
                     [math.sin(yaw), math.cos(yaw)]])
    # car = np.dot(rot1, car)
    # car += np.array([[x], [y]])
    polygon_msg = PolygonStamped()
    polygon_msg.header.frame_id = 'base_link'
    polygon_msg.polygon.points = [
        Point32(x=car[0][0], y=car[1][0], z=0.0),
        Point32(x=car[0][1], y=car[1][1], z=0.0),
        Point32(x=car[0][2], y=car[1][2], z=0.0),
        Point32(x=car[0][3], y=car[1][3], z=0.0),
        Point32(x=car[0][4], y=car[1][4], z=0.0),
        Point32(x=car[0][5], y=car[1][5], z=0.0),
        Point32(x=car[0][6], y=car[1][6], z=0.0),
        Point32(x=car[0][7], y=car[1][7], z=0.0),
    ]
    polygon_pub_.publish(polygon_msg)
    polygon_points = [(car[0][0], car[1][0]), 
                      (car[0][1], car[1][1]), 
                      (car[0][2], car[1][2]), 
                      (car[0][3], car[1][3]),
                      (car[0][4], car[1][4]),
                      (car[0][5], car[1][5]),
                      (car[0][6], car[1][6]),
                      (car[0][7], car[1][7])]
    polygon = Polygon(polygon_points)

    collision_msg = Bool()
    collision_msg.data = False

    # 遍历激光雷达的距离数据，并将其转换为笛卡尔坐标
    ranges = msg.ranges  # 获取激光雷达的距离数据
    angle_min = msg.angle_min  # 激光雷达的最小角度
    angle_increment = msg.angle_increment  # 激光雷达的角度增量
    points = []  # 用来存储转换后的坐标点
    collision_points = [] # 用来存储碰撞点
    for i, r in enumerate(ranges):
        if r > msg.range_min and r < msg.range_max:  # 检查有效的距离
            # 计算激光束对应的角度
            angle = angle_min + i * angle_increment
            # 将极坐标转换为笛卡尔坐标
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            points.append((x, y))  # 添加到点列表中
            point = Point(x, y)
            if polygon.contains(point):
                collision_points.append((x, y))
                collision_msg.data = True
            else:
                points.append((x, y))

    print(len(collision_points))
   
    
    collision_pub_.publish(collision_msg)

    # 创建 MarkerArray 消息，用于发布多个点
    marker_array = MarkerArray()
    for i, (x, y) in enumerate(points):
        marker = Marker()
        marker.header.frame_id = "base_link"  # 设置参考坐标系
        marker.header.stamp = rospy.Time.now()
        marker.ns = "points"
        marker.id = i  # 每个点的唯一标识符
        marker.type = Marker.SPHERE  # 使用球体表示点
        marker.action = Marker.ADD
        marker.pose.position = Point(x, y, 0)  # 设置点的位置
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.01  # 设置球体的半径
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.a = 1.0  # 设置透明度（1.0表示不透明）
        marker.color.g = 1.0  # 设置颜色（红色）
        
        # 将 Marker 添加到 MarkerArray
        marker_array.markers.append(marker)

    for i, (x, y) in enumerate(collision_points):
        marker = Marker()
        marker.header.frame_id = "base_link"  # 设置参考坐标系
        marker.header.stamp = rospy.Time.now()
        marker.ns = "points"
        marker.id = len(points) + i  # 每个点的唯一标识符
        marker.type = Marker.SPHERE  # 使用球体表示点
        marker.action = Marker.ADD
        marker.pose.position = Point(x, y, 0)  # 设置点的位置
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.01  # 设置球体的半径
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.a = 1.0  # 设置透明度（1.0表示不透明）
        marker.color.r = 1.0  # 设置颜色（红色）
        
        # 将 Marker 添加到 MarkerArray
        marker_array.markers.append(marker)
    marker_pub.publish(marker_array)
        
def _sunny_state_callback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    euler = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    yaw = euler[2]
    # print(x, y, yaw)

    # x = 0
    # y = 0
   


if __name__=="__main__":

    # laser_scan = None
    rospy.init_node("ros_rl", anonymous=True)#初始化节点 名称：test
    polygon_pub_ = rospy.Publisher('/polygon', PolygonStamped, queue_size=10)
    marker_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
    collision_pub_ = rospy.Publisher('/collision', Bool, queue_size=10)
    # logger.info("Waiting for /scan to be READY...")
    # rospy.Subscriber("/scan", Twist, _laser_scan_callback)
    # while laser_scan is None and not rospy.is_shutdown():
    #     try:
    # laser_scan = rospy.wait_for_message("/sunny/state", Twist, timeout=5.0)
    #         rospy.logdebug("Current /scan READY=>")

    #     except:
    #         rospy.logerr("Current /scan not ready yet, retrying for getting laser_scan")
    # print(laser_scan)
    rospy.Subscriber("/top_scan", LaserScan, _top_laser_scan_callback)
    # rospy.Subscriber("/sunny/state", Odometry, _sunny_state_callback)

    rospy.spin()

