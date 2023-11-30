#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
import tf
import numpy as np
import time
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32 as Map_Match_Ratio


class MapScanMatcher:
    def __init__(self):
        rospy.init_node('map_scan_matcher')

        # 訂閱Lidar和地圖數據
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.Map_Match_Ratio_pub = rospy.Publisher("/map_match_ratio", Map_Match_Ratio, queue_size=10)
        
        self.latest_scan = None
        self.map_data = None
        self.listener = tf.TransformListener()

    def scan_callback(self, scan):
        self.latest_scan = scan

    def map_callback(self, map_data):
        self.map_data = map_data

    def process_scan(self):
        if self.latest_scan and self.map_data:
            try:
                # 獲取轉換
                (trans, rot) = self.listener.lookupTransform('/map', self.latest_scan.header.frame_id, rospy.Time(0))

                matched_points = 0
                total_points = 0

                for angle, distance in enumerate(self.latest_scan.ranges):
                    if distance == float('Inf') or distance == 0.0:
                        continue

                    # 計算掃描點在車輛坐標系中的位置
                    angle_rad = self.latest_scan.angle_min + angle * self.latest_scan.angle_increment
                    x = distance * np.cos(angle_rad)
                    y = distance * np.sin(angle_rad)

                    # 轉換到地圖坐標系
                    point = PointStamped()
                    point.header.frame_id = self.latest_scan.header.frame_id
                    point.point.x = x
                    point.point.y = y
                    transformed_point = self.listener.transformPoint('/map', point)

                    # 將點轉換為地圖上的格子索引
                    map_x = int((transformed_point.point.x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
                    map_y = int((transformed_point.point.y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)
                    # rospy.loginfo("Map Index: ({}, {}, {}, {})".format(x, y, map_x, map_y))

                    # 檢查該點在地圖上的值
                    if 0 <= map_x < self.map_data.info.width and 0 <= map_y < self.map_data.info.height:
                        map_index = map_y * self.map_data.info.width + map_x
                        if self.map_data.data[map_index] == 100:  # 0 表示可通行區域
                            matched_points += 1
                        total_points += 1

                if total_points > 0:
                    match_ratio = float(matched_points) / total_points
                    # rospy.loginfo("Match Ratio: {:.2f}".format(match_ratio))
                    self.Map_Match_Ratio_pub.publish(match_ratio)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logerr("TF轉換錯誤: %s", e)

if __name__ == '__main__':
    time.sleep(12.0) #wait for rviz map to be ready
    try:
        matcher = MapScanMatcher()
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            matcher.process_scan()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass


