#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import csv
import rospy
import tf.transformations
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseArray as multgoals_Msg
from visualization_msgs.msg import Marker
from std_msgs.msg import Int8 as Navigation_control_Msg 
from std_msgs.msg import Int8 as Stop_Navigation_control_Msg
from std_msgs.msg import Int16 as Relocation_initpose_Msg
from std_msgs.msg import String as Navigation_Status_Msg
from std_msgs.msg import UInt8 as button_status_msg
from std_msgs.msg import UInt8 as onesite_arrived_msg
from geometry_msgs.msg import Twist


def navigation():
    markerPub = rospy.Publisher('goal_point', Marker, queue_size=1)
    goalPub = rospy.Publisher('current_goal', Pose, queue_size=1)

    # Read saved points
    fileName = rospy.get_param('~saved_points_file', '')
    global control_signal
    goals = []

    try:
        with open(fileName) as csvfile: 
            rows = csv.reader(csvfile, delimiter=',')  # read with comma as separator
            next(rows)  # Skip first
            for row in rows: 
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.pose.position.x = float(row[1])
                goal.target_pose.pose.position.y = float(row[2])
                goal.target_pose.pose.position.z = 0
                goal.target_pose.pose.orientation.x = float(row[3])
                goal.target_pose.pose.orientation.y = float(row[4])
                goal.target_pose.pose.orientation.z = float(row[5])
                goal.target_pose.pose.orientation.w = float(row[6])
                goals.append(goal)
    except Exception as e: 
        print(e)

    # Publish all goal points
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.header.stamp = rospy.Time.now()
    marker.id = 0
    marker.type = Marker.POINTS
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.scale.x = 0.25
    marker.scale.y = 0.25

    for goal in goals:
        marker.points.append(goal.target_pose.pose.position)

    markerPub.publish(marker)

    # Wait for move_base to initialize
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
  
    # Move to all goals
    for goal in goals: 

        goal.target_pose.header.stamp = rospy.Time.now()
        client.send_goal(goal)
        state = client.get_state()
        rospy.loginfo("state:  " + str(state))
        
        # Use a loop to continuously check for control_signal while moving to the goal
        while not client.get_state() in [actionlib.GoalStatus.SUCCEEDED, actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.PREEMPTED]:
            if control_signal == "34":
                stoppath_navigation()
                rospy.loginfo("Interrupted and going back to origin")
                return
            rospy.sleep(0.1)
            
        # Check if arrived at the goal or if there was an error
        if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo('Goal Arrived, moving to next goal.')
        else:
            rospy.loginfo('Error reaching goal.')
    rospy.loginfo('All goals arrived, going home.')    


def stoppath_navigation():
    onesite_arrived = onesite_arrived_msg()
    onesite_arrived.data = 0xBC  #arduino led red light
    Onesite_arrived_pub.publish(onesite_arrived)

    stopclient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.sleep(1)
    stopclient.cancel_all_goals()

    stopclient.wait_for_result()
    # state = client.get_state()
    # rospy.loginfo("state:  " + str(state))

    stop_cmd_Pub = rospy.Publisher('omni_base_driver/cmd_vel', Twist, queue_size=2)  
    stop_cmd = Twist()
    stop_cmd_Pub.publish(stop_cmd)  

    home = MoveBaseGoal()
    home.target_pose.header.stamp = rospy.Time.now()
    home.target_pose.header.frame_id = 'map'
    home.target_pose.pose.position.x = 0
    home.target_pose.pose.position.y = 0
    home.target_pose.pose.position.z = 0
    home.target_pose.pose.orientation.x = 0
    home.target_pose.pose.orientation.y = 0
    home.target_pose.pose.orientation.z = 0
    home.target_pose.pose.orientation.w = 1
    stopclient.send_goal(home)


    stopclient.wait_for_result() 
    result = stopclient.get_state()

    global control_signal
    control_signal = None

    return result == actionlib.GoalStatus.SUCCEEDED
      

def Initposecallback(initialize_Relocation):
    global Relocation_init
    Relocation_init = initialize_Relocation.data
    rospy.loginfo("initialize_Relocation: %s", str(Relocation_init))
    if(Relocation_init == 38):
        initpose = PoseWithCovarianceStamped()
        initpose.header.frame_id = 'map'
        initpose.header.stamp = rospy.Time.now()
        
        initPoint = Point()
        initPoint.x = 0
        initPoint.y = 0
        initPoint.z = 0
        
        initpose.pose.pose.position = initPoint

        quat = tf.transformations.quaternion_from_euler(0, 0, initPoint.z)
        initpose.pose.pose.orientation.x = quat[0]
        initpose.pose.pose.orientation.y = quat[1]
        initpose.pose.pose.orientation.z = quat[2]
        initpose.pose.pose.orientation.w = quat[3]
        yaw = tf.transformations.euler_from_quaternion(quat)[2]
        print("(x, y, z) = ({0}, {1}, {2})".format(initpose.pose.pose.position.x, initpose.pose.pose.position.y, yaw))

        Relocation_initpose_pub.publish(initpose)
        Relocation_init = None
    

def NavigationCallback(navigationSend):
    global control_signal 
    control_signal = str(navigationSend.data)
    rospy.loginfo("navigationSend: %s", control_signal)


def Stop_Navigation_control(stopnavigationSend):
    global control_signal
    control_signal = str(stopnavigationSend.data)
    rospy.loginfo("stopnavigationSend: %s", control_signal)

# multiple goals navigation
def multiple_goals_navigation(pose_array_msg):
    onesite_arrived = onesite_arrived_msg()
    goals = []
    global control_signal
    goal_count = -1  # start site is site 0
    for pose in pose_array_msg.poses: 
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose  # Directly assign the pose received from the PoseArray
        goals.append(goal)
    rospy.loginfo("Prepared goals for navigation: %s", goals)
    
    # Wait for move_base to initialize
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    Navigation_Status_pub.publish("Receive navgation goals, prepare to navigation.")

    # Move to all goals
    for goal in goals: 
        goal_count += 1
        
        if goal_count == 0 and button_status != 0x00:
            continue  # 如果button_status不是0x00，则跳过此目标

        # 当goal_count>0时，需要等待button_status变为0xAA才继续移动
        elif goal_count > 0:
            start_time = rospy.Time.now()
            Navigation_Status_pub.publish("Waiting for Recipient check things and Pressed button for next goal " + str(goal_count))
            while button_status != 0xAA:
                Navigation_Status_pub.publish("Waiting for 30sec or Waiting Recipient take things Pressed button for next goal " + str(goal_count))
                if (rospy.Time.now() - start_time).to_sec() > 30:
                    Navigation_Status_pub.publish("Waiting for 30sec, Recipent didn't press button, going next goal")
                    break
                elif control_signal == "34":
                    break
                rospy.sleep(0.1)  # 等待button_status更新为0xAA

        if control_signal == "34":
            Navigation_Status_pub.publish("Interrupted Stie " + str(goal_count) + " and going back to origin")
            if stoppath_navigation():
                onesite_arrived.data = 0xBD  #arduino led red off
                Onesite_arrived_pub.publish(onesite_arrived)
                Navigation_Status_pub.publish("back to origin")
            return

        rospy.loginfo("Proceeding to goal " + str(goal_count))

        goal.target_pose.header.stamp = rospy.Time.now()
        client.send_goal(goal)
        state = client.get_state()
        rospy.loginfo("state:  " + str(state))
        onesite_arrived.data = 0xBD
        Onesite_arrived_pub.publish(onesite_arrived)
        Navigation_Status_pub.publish("Moving...." + " Goal : Site" + str(goal_count))
        
        # Use a loop to continuously check for control_signal while moving to the goal
        while not client.get_state() in [actionlib.GoalStatus.SUCCEEDED, actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.PREEMPTED]:
            rospy.sleep(0.1)
            
        # Check if arrived at the goal or if there was an error
        if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            Navigation_Status_pub.publish("Goal Arrived Site " + str(goal_count) + ", moving to next goal.")
            onesite_arrived.data = 0xBB #arduino led green light
            Onesite_arrived_pub.publish(onesite_arrived)
        else:
            Navigation_Status_pub.publish("Error reaching goal Site " + str(goal_count))
    
    Navigation_Status_pub.publish('All goals arrived.')
    onesite_arrived.data = 0xBD
    Onesite_arrived_pub.publish(onesite_arrived)

  
def button_receiver_callback(button_receiver):
    global button_status
    button_status = (button_receiver.data)
    # rospy.loginfo("button_status: %s", button_status)


if __name__ == '__main__':
    try:
        rospy.init_node('demo')
        #subscribers-----------------------------------------------------------------------------------------------------------------
        navigation_sub = rospy.Subscriber('/navigationSend', Navigation_control_Msg, NavigationCallback, queue_size=1)
        stop_navigation_sub = rospy.Subscriber('/stopnavigationSend', Stop_Navigation_control_Msg, Stop_Navigation_control, queue_size=1)
        multiple_goals_sub = rospy.Subscriber('/multiple_goals', multgoals_Msg, multiple_goals_navigation, queue_size=20)
        initialize_Relocation_sub = rospy.Subscriber('/initialize_Relocation', Relocation_initpose_Msg, Initposecallback, queue_size=1)
        button_receiver_sub = rospy.Subscriber('/button_receiver', button_status_msg, button_receiver_callback, queue_size=1)

        #publishers------------------------------------------------------------------------------------------------------------------
        Relocation_initpose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        Navigation_Status_pub = rospy.Publisher('/navigation_status', Navigation_Status_Msg, queue_size=20)
        Onesite_arrived_pub = rospy.Publisher('/onesite_arrived', onesite_arrived_msg, queue_size=2)

        #global variables-------------------------------------------------------------------------------------------------------------
        control_signal = None
        Relocation_init = None
        button_status = None

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            # joystick control
            if control_signal == "8":
                navigation()
                control_signal = None
        
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass
