#!/usr/bin/env python3

import rospy
import yaml
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class waypoints_publisher:

    def __init__(self):
        rospy.init_node('waypoints_publisher')
        rate = rospy.Rate(10) # 10hz
        self.marker_pub = rospy.Publisher('/waypoints', MarkerArray, queue_size=10)
        waypoint_file = rospy.get_param('~waypoint_file')
        
        with open(waypoint_file, 'r') as f:
            self.point_datas = yaml.load(f, Loader=yaml.FullLoader)

        while not rospy.is_shutdown():           
            self.publish_waypoints()
            rate.sleep()


    def publish_waypoints(self):
        markers = []              
        
        markers.extend(self.generate_marker('init', self.point_datas['Initial_Pose'], color=(1,0,0,0.8)))
        markers.extend(self.generate_marker('basketball_rack_ready', self.point_datas['Basketball']['rack']['ready_pose'], color=(0,0,1,0.8), text='rack_ready'))
        # markers.extend(self.generate_marker('basketball_rack_ready', self.point_datas['Basketball']['rack']['ready_pose_2'], color=(0,0,1,0.8), text='rack_ready_2'))
        markers.extend(self.generate_marker('basketball_rack', self.point_datas['Basketball']['rack']['pose'], color=(0,0,1,0.8), text='rack'))
        markers.extend(self.generate_marker('basketball_throw_ready', self.point_datas['Basketball']['throw']['ready_pose'], color=(0,0,1,0.8), text='throw_ready'))
        # markers.extend(self.generate_marker('basketball_throw_ready', self.point_datas['Basketball']['throw']['ready_pose_2'], color=(0,0,1,0.8), text='throw_ready_2'))
        # markers.extend(self.generate_marker('basketball_throw_ready', self.point_datas['Basketball']['throw']['ready_pose_3'], color=(0,0,1,0.8), text='throw_ready_3'))
        markers.extend(self.generate_marker('basketball_throw_1', self.point_datas['Basketball']['throw']['poses'][0], color=(0,0,1,0.8), text='1'))
        markers.extend(self.generate_marker('basketball_throw_2', self.point_datas['Basketball']['throw']['poses'][1], color=(0,0,1,0.8), text='2'))
        markers.extend(self.generate_marker('basketball_throw_3', self.point_datas['Basketball']['throw']['poses'][2], color=(0,0,1,0.8), text='3'))

        markers.extend(self.generate_marker('bowling_ready', self.point_datas['Bowling']['ready_pose'], color=(0,1,0,0.8), text='ready'))
        markers.extend(self.generate_marker('bowling_rack', self.point_datas['Bowling']['rack']['pose'], color=(0,1,0,0.8), text='rack'))        
        markers.extend(self.generate_marker('bowling_throw_1', self.point_datas['Bowling']['throw']['poses'][0], color=(0,1,0,0.8), text='1'))
        markers.extend(self.generate_marker('bowling_throw_2', self.point_datas['Bowling']['throw']['poses'][1], color=(0,1,0,0.8), text='2'))
        markers.extend(self.generate_marker('bowling_throw_3', self.point_datas['Bowling']['throw']['poses'][2], color=(0,1,0,0.8), text='3'))
        markers.extend(self.generate_marker('bowling_throw_4', self.point_datas['Bowling']['throw']['poses'][3], color=(0,1,0,0.8), text='4'))
        markers.extend(self.generate_marker('bowling_throw_5', self.point_datas['Bowling']['throw']['poses'][4], color=(0,1,0,0.8), text='5'))
        markers.extend(self.generate_marker('bowling_throw_6', self.point_datas['Bowling']['throw']['poses'][5], color=(0,1,0,0.8), text='6'))
        
        self.marker_pub.publish(markers)

    def generate_marker(self, name, pose, color=(0.0,0.0,0.0,1.0), text=''):
        markers = []        

        marker = Marker()
        marker.ns = name
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time()
        marker.id = 0
        marker.type = Marker.ARROW
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.scale.x = 0.3
        marker.scale.y = 0.32
        marker.scale.z = 0.1
        marker.pose.position.x = pose['x']
        marker.pose.position.y = pose['y']
        marker.pose.position.z = 0
        marker.pose.orientation.z = pose['qz']
        marker.pose.orientation.w = pose['qw']                
        markers.append(marker)  

        marker = Marker()
        marker.ns = name + '_text'
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time()
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        marker.scale.z = 0.5
        marker.pose.position.x = pose['x'] + 0.5
        marker.pose.position.y = pose['y']
        marker.pose.position.z = 0
        if text == '':
            text = name
        marker.text = text
        markers.append(marker)  

        return markers


if __name__ == '__main__':
    try:
        waypoints_publisher()
    except rospy.ROSInterruptException:
        pass
