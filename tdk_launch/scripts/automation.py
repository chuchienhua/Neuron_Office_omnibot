#!/usr/bin/env python3
from operator import truediv
from os import stat
from turtle import st
import turtle
import rospy
import roslaunch
import serial
import threading
import yaml
import os
from enum import Enum

import actionlib        # https://docs.ros.org/en/diamondback/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

BASKETBALL_GET_TIMEOUT = 60.0         # 60 secs
BASKETBALL_THROW_TIMEOUT = 30.0       # 30 secs
BOWLING_BALL_GET_TIMEOUT = 60.0       # 40 secs
BOWLING_BALL_THROW_TIMEOUT = 30.0     # 30 secs
 
# Enums
class CtrlState(Enum):
    IDLE = b'idle\n'
    RUNNING_A = b'running A\n'
    RUNNING_B = b'running B\n'
    ERROR = b'error\n'

class Action(Enum):
    NONE = ''
    START_A = b'start A\n'
    START_B = b'start B\n'
    STOP = b'stop\n'

class ArmAction(Enum):
    NONE = ''
    ARM_START = b'1,0\n'
    GET_BASKETBALL = b'3,0\n'
    THROW_BASKETBALL = b'23,0\n'
    GET_BOWLING = b'5,0\n'
    THROW_BOWLING1 = b'7,0\n'
    THROW_BOWLING2 = b'21,0\n'
    THROW_BOWLING3 = b'22,0\n'

ctrl_state = CtrlState.IDLE
action = Action.NONE
arm_action = ArmAction.NONE
arm_read = ''
arrived = False
arm_finish_state = False
arm_last_action_time = 0
prev_stage = 0
stage = 0   #  0: Idle
            #  1: Going to basketball rack ready position (TEB)
            #  2: Going to basketball rack  (PP Strict *Pure Pursuit)
            #  3: Acquiring basketballs
            #  4: Going back to basketball rack ready position (PP Loose)
            #  5: Going to basketball throwing ready position (TEB)
            #  6: Going to basketball throwing positions and throw* (PP Strict)
            #  7: Going back to basketball throwing ready position (PP Loose)
            #  8: Going to bowling ready position (TEB)
            #  9: Going to bowling ball rack (PP Strict)
            # 10: Acquiring bowling balls
            # 11: Going back to bowling ready position (PP Loose)
            # 12: Going to bowling ball throwing position1 and throw (PP Strict)
            # 13: Going to bowling ball throwing position2 and throw (PP Strict)
            # 14: Going to bowling ball throwing position3 and throw (PP Strict)
            # 15: Done
basketball_prev_stage = 0
bowling_ball_prev_stage = 0
basketball_stage = 0
bowling_ball_stage = 0

basketball_throw_index1 = 0
basketball_throw_index2 = 0
basketball_throw_index3 = 0
bowling_throw_index1 = 0
bowling_throw_index2 = 0
bowling_throw_index3 = 0
    

# Communicate with TDK control box
def tdk_ctrl_box_com():
    global ctrl_state
    global action
    ser = serial.Serial(port='/dev/tdk', baudrate=115200,timeout=2,rtscts=True,dsrdtr=True)
    ser.isOpen()
    while not rospy.is_shutdown():
        ser.write(ctrl_state.value)        

        if ser.in_waiting:
            res = ser.readline()
            action = Action(res)            
        rospy.sleep(0.1)
    
    ser.close()

# Communicate with Teensy_arm
def robot_arm_com():
    global arm_finish_state
    global arm_action
    global arm_read

    ser_arm = serial.Serial(port='/dev/teensy_arm', baudrate=115200,timeout=2,rtscts=True,dsrdtr=True)
    ser_arm.isOpen()

    while not rospy.is_shutdown(): 
        if arm_action != ArmAction.NONE:
            ser_arm.write(arm_action.value)
            print(arm_action)
            arm_finish_state = False
            arm_action = ArmAction.NONE

        if ser_arm.in_waiting:
            arm_read = ser_arm.readline()
            if arm_read == b'\n':
                arm_finish_state = True
                arm_read = ''
            else:
                arm_finish_state = False
        rospy.sleep(0.1)
    
    ser_arm.close()


def automation():    
    global ctrl_state
    global action
    global arm_action
    global arrived
    global stage
    global prev_stage
    global basketball_stage
    global basketball_prev_stage
    global bowling_ball_stage
    global bowling_ball_prev_stage

    global arm_finish_state
    global arm_read
    global basketball_throw_index1
    global basketball_throw_index2
    global basketball_throw_index3
    global bowling_throw_index1
    global bowling_throw_index2
    global bowling_throw_index3

    rospy.init_node('automation')
    rate = rospy.Rate(10) # 10hz

    # Start TDK control box communication
    t = threading.Thread(target=tdk_ctrl_box_com)
    t.start()
    # Start Robot arm communication
    t_arm = threading.Thread(target=robot_arm_com)
    t_arm.start()

    waypoint_A_file = rospy.get_param('~waypoint_A_file')
    waypoint_B_file = rospy.get_param('~waypoint_B_file')
    map_A_file = rospy.get_param('~map_A_file')
    map_B_file = rospy.get_param('~map_B_file')
    visualization = rospy.get_param('~visualization')

    # Pure pursuit publisher and subscriber
    waypoint_pub = rospy.Publisher('/tdk_pure_pursuit/target', Pose, queue_size=1, tcp_nodelay=True) 
    strict_pub = rospy.Publisher('/tdk_pure_pursuit/strict', Bool, queue_size=1, tcp_nodelay=True)
    rospy.Subscriber("/tdk_pure_pursuit/arrived", Bool, arrived_callback, tcp_nodelay=True)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    cli_args_A = ['tdk_navigation', 'tdk_navigation.launch', 'map_file:=%s' % (map_A_file), 'open_rviz:=%s' % (visualization)]
    roslaunch_args_A = cli_args_A[2:]
    roslaunch_file_A = [(roslaunch.rlutil.resolve_launch_arguments(cli_args_A)[0], roslaunch_args_A)]

    cli_args_B = ['tdk_navigation', 'tdk_navigation.launch', 'map_file:=%s' % (map_B_file), 'open_rviz:=%s' % (visualization)]
    roslaunch_args_B = cli_args_B[2:]
    roslaunch_file_B = [(roslaunch.rlutil.resolve_launch_arguments(cli_args_B)[0], roslaunch_args_B)]
    

    with open(waypoint_A_file, 'r') as f:
        A_point_datas = yaml.load(f, Loader=yaml.FullLoader)
    with open(waypoint_B_file, 'r') as f:
        B_point_datas = yaml.load(f, Loader=yaml.FullLoader)

    while not rospy.is_shutdown():
        # Determine control box actions
        if action == Action.START_A:
            if ctrl_state == CtrlState.IDLE:
                ctrl_state = CtrlState.RUNNING_A 
                rospy.loginfo("Starting Map A")
                point_datas = A_point_datas
                # Start tdk_navigation.launch
                parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file_A)
                parent.start()

                # Initialize Robot arm
                rospy.loginfo("Initializing robot arm...")
                arm_action = ArmAction.ARM_START
                arm_last_action_time = rospy.get_time()

                # Wait for move_base to initialize
                rospy.loginfo("Waiting Move Base...")
                teb_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
                teb_client.wait_for_server()
                rospy.loginfo("Move Base Initialized!")
                stage = 1
                basketball_stage = 1
                bowling_ball_stage = 1
                action = Action.NONE
        elif action == Action.START_B:
            if ctrl_state == CtrlState.IDLE:
                ctrl_state = CtrlState.RUNNING_B    
                rospy.loginfo("Starting Map B")
                point_datas = B_point_datas
                # Start tdk_navigation.launch
                parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file_B)
                parent.start()

                # Initialize Robot arm
                rospy.loginfo("Initializing robot arm...")
                arm_action = ArmAction.ARM_START
                arm_last_action_time = rospy.get_time()   

                # Wait for move_base to initialize
                rospy.loginfo("Waiting Move Base...")
                teb_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
                teb_client.wait_for_server()
                rospy.loginfo("Move Base Initialized!")
                stage = 1
                basketball_stage = 1
                bowling_ball_stage = 1
                action = Action.NONE
        elif action == Action.STOP:
            if ctrl_state != CtrlState.IDLE:
                ctrl_state = CtrlState.IDLE
                parent.shutdown()
                stage = 0
                prev_stage = 0
                basketball_stage = 0
                bowling_ball_stage = 0
                action = Action.NONE
        
        #  1: Going to basketball rack ready position (TEB)
        if stage == 1:
            if prev_stage == stage-1:
                rospy.loginfo("Starting stage 1:")
                goal = get_goal(point_datas['Basketball']['rack']['ready_pose'])
                teb_client.send_goal(goal)
            else:                        
                state = teb_client.get_state()      
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Stage 1 done")           
                    stage = 2
        
        #  2: Going to basketball rack  (PP Strict *Pure Pursuit)
        if stage == 2:
            if prev_stage == stage-1:
                rospy.loginfo("Starting stage 2:")
                target_pose = get_target_pose(point_datas['Basketball']['rack']['pose'])           
                waypoint_pub.publish(target_pose)
                strict_pub.publish(Bool(True))
                arrived = False
            else:
                if arrived:
                    rospy.loginfo("Stage 2 done")
                    stage = 3

        #  3: Acquiring basketballs
        if stage == 3:            
            if prev_stage == stage-1:
                rospy.loginfo("Starting stage 3:")
                # Implement basketball acquiring code here
                rospy.loginfo("Getting:")
                arm_action = ArmAction.GET_BASKETBALL
                arm_last_action_time = rospy.get_time()                           
                # Implement basketball acquiring code here    
            else:   
                if arm_finish_state:
                    print(arm_read)                    
                    rospy.loginfo("Stage 3 done")                 
                    stage = 4
                elif rospy.get_time() - arm_last_action_time > BASKETBALL_GET_TIMEOUT:
                    print(rospy.get_time())
                    print(arm_last_action_time)
                    rospy.loginfo("Stage 3 done due to timeout")                 
                    stage = 4

        #  4: Going back to basketball rack ready position (PP Loose)
        if stage == 4:
            if prev_stage == stage-1:
                rospy.loginfo("Starting stage 4:")
                target_pose = get_target_pose(point_datas['Basketball']['rack']['ready_pose'])            
                waypoint_pub.publish(target_pose)
                strict_pub.publish(Bool(False))
                arrived = False
            else:
                if arrived:
                    rospy.loginfo("Stage 4 done")
                    stage = 5

        #  5: Going to basketball throwing ready position (TEB)
        if stage == 5:
            if prev_stage == stage-1:
                rospy.loginfo("Starting stage 5:")
                goal = get_goal(point_datas['Basketball']['throw']['ready_pose'])
                teb_client.send_goal(goal)
            else:
                state = teb_client.get_state()            
                if state == GoalStatus.SUCCEEDED:                    
                    rospy.loginfo("Stage 5 done")
                    stage = 6
                    
        #  6: Going to basketball throwing ready position 2 (TEB)
        if stage == 6:
            if prev_stage == stage-1:
                rospy.loginfo("Starting stage 6:")
                goal = get_goal(point_datas['Basketball']['throw']['ready_pose_2'])
                teb_client.send_goal(goal)
            else:
                state = teb_client.get_state()            
                if state == GoalStatus.SUCCEEDED:                    
                    rospy.loginfo("Stage 6 done")
                    stage = 7

        #  7: Going to basketball throwing positions and throw* (PP Strict)
        if stage == 7:
            if prev_stage == stage-1:
                rospy.loginfo("Starting stage 7:")
                target_pose = get_target_pose(point_datas['Basketball']['throw']['poses'][1])   # Middle                
                waypoint_pub.publish(target_pose)
                strict_pub.publish(Bool(True))
                arrived = False
                throw_ball = False
            else:
                if arrived and not throw_ball:
                    arm_action = ArmAction.THROW_BASKETBALL
                    arm_last_action_time = rospy.get_time()
                    throw_ball= True
                elif arrived and throw_ball:
                    
                    if arm_finish_state:
                        print(arm_read)
                        rospy.loginfo("Stage 7 done")
                        stage = 8          
                    elif rospy.get_time() - arm_last_action_time > BASKETBALL_THROW_TIMEOUT:
                        print(rospy.get_time)
                        print(arm_last_action_time)
                        rospy.loginfo("Stage 7 done due to timeout")
                        stage = 8
                                               

        #  8: Going back to basketball throwing ready position 2(PP Loose)
        if stage == 8:
            if prev_stage == stage-1:
                rospy.loginfo("Starting stage 8:")
                target_pose = get_target_pose(point_datas['Basketball']['throw']['ready_pose_2'])               
                waypoint_pub.publish(target_pose)
                strict_pub.publish(Bool(False))
                arrived = False
            else:
                if arrived:
                    rospy.loginfo("Stage 8 done")
                    stage = 9
        
        #  9: Going to basketball throwing ready position 3 (TEB)
        if stage == 9:
            if prev_stage == stage-1:
                rospy.loginfo("Starting stage 9:")
                goal = get_goal(point_datas['Basketball']['throw']['ready_pose_3'])
                teb_client.send_goal(goal)
            else:
                state = teb_client.get_state()            
                if state == GoalStatus.SUCCEEDED:                    
                    rospy.loginfo("Stage 9 done")
                    stage = 10

        # 10: Going to bowling ready position (TEB)            
        if stage == 10:
            if prev_stage == stage-1:
                rospy.loginfo("Starting stage 10:")
                goal = get_goal(point_datas['Bowling']['ready_pose'])
                teb_client.send_goal(goal)
            else:
                state = teb_client.get_state()            
                if state == GoalStatus.SUCCEEDED:
                    f = os.popen('python3 /home/tdk/tdk/tdk_ros_ws/src/tdk_launch/recog_scripts/Braille/braille.py')
                    new_list = []
                    data = f.read()  
                    for i in range(len(data)) :
                        if i % 5 == 2:
                            new_list.append(data[i])

                    bowling_throw_index1 = new_list.index('1')
                    bowling_throw_index2 = new_list.index('2')
                    bowling_throw_index3 = new_list.index('3')
                    f.close()
                    rospy.loginfo("Stage 10 done")
                    stage = 11

        #  11: Going to bowling ball rack (PP Strict)
        if stage == 11:
            if prev_stage == stage-1:
                rospy.loginfo("Starting stage 11:")
                target_pose = get_target_pose(point_datas['Bowling']['rack']['pose'])           
                waypoint_pub.publish(target_pose)
                strict_pub.publish(Bool(True))
                arrived = False
            else:
                if arrived:
                    rospy.loginfo("Stage 11 done")                 
                    stage = 12

        # 12: Acquiring bowling balls
        if stage == 12:
            if prev_stage == stage-1:
                rospy.loginfo("Starting stage 12:")
                # Implement bowling acquiring code here
                arm_action = ArmAction.GET_BOWLING
                arm_last_action_time = rospy.get_time()
                # Implement bowling acquiring code here
            else:
                if arm_finish_state:
                    print(arm_read) 
                    rospy.loginfo("Stage 12 done")                 
                    stage = 13
                elif rospy.get_time() - arm_last_action_time > BOWLING_BALL_GET_TIMEOUT:
                    print(rospy.get_time())
                    print(arm_last_action_time)
                    rospy.loginfo("Stage 12 done due to timeout")                 
                    stage = 13

        # 13: Going back to bowling ready position (PP Loose)
        if stage == 13:
            if prev_stage == stage-1:
                rospy.loginfo("Starting stage 13:")
                target_pose = get_target_pose(point_datas['Bowling']['ready_pose'])            
                waypoint_pub.publish(target_pose)
                strict_pub.publish(Bool(False))
                arrived = False
            else:
                if arrived:
                    rospy.loginfo("Stage 13 done")
                    stage = 14

        # 14: Going to bowling ball throwing positions and throw* (PP Strict)
        if stage == 14:
            if prev_stage == stage-1:
                rospy.loginfo("Starting stage 14:")
                target_pose = get_target_pose(point_datas['Bowling']['throw']['poses'][5-bowling_throw_index1])   # Braille 1                
                waypoint_pub.publish(target_pose)
                strict_pub.publish(Bool(True))
                arrived = False
                throw_ball = False
            else:
                if arrived and not throw_ball:
                        rospy.loginfo("Arrived. Start to throw bowling ball 1")
                        # Implement bowling ball throwing code here
                        arm_action = ArmAction.THROW_BOWLING1
                        arm_last_action_time = rospy.get_time()
                        throw_ball = True
                elif arrived and throw_ball:
                    if arm_finish_state:
                        print(arm_read) 
                        # Implement bowling ball throwing code here
                        rospy.loginfo("Stage 14 done")
                        stage = 15
                    elif rospy.get_time() - arm_last_action_time > BOWLING_BALL_THROW_TIMEOUT:
                        print(rospy.get_time())
                        print(arm_last_action_time)
                        rospy.loginfo("Stage 14 done due to timeout")           
                        stage = 15
        
        # 15: Going to bowling ball throwing position2 and throw (PP Strict)
        if stage == 15:
            if prev_stage == stage-1:
                rospy.loginfo("Starting stage 15:")
                target_pose = get_target_pose(point_datas['Bowling']['throw']['poses'][5-bowling_throw_index2])   # Braille 2               
                waypoint_pub.publish(target_pose)
                strict_pub.publish(Bool(True))
                arrived = False
                throw_ball = False
            else:
                if arrived:
                    if arrived and not throw_ball:
                        rospy.loginfo("Arrived. Start to throw bowling ball 2")
                        # Implement bowling ball throwing code here
                        arm_action = ArmAction.THROW_BOWLING2
                        arm_last_action_time = rospy.get_time()
                        throw_ball = True
                    elif arrived and throw_ball:
                        if arm_finish_state:
                            print(arm_read)
                            rospy.loginfo("Stage 15 done")
                            stage = 16
                        elif rospy.get_time() - arm_last_action_time > BOWLING_BALL_THROW_TIMEOUT:
                            print(rospy.get_time())
                            print(arm_last_action_time)
                            rospy.loginfo("Stage 15 done due to timeout")           
                            stage = 16
        
        # 16: Going to bowling ball throwing position3 and throw (PP Strict)
        if stage == 16:
            if prev_stage == stage-1:
                rospy.loginfo("Starting stage 16:")
                target_pose = get_target_pose(point_datas['Bowling']['throw']['poses'][5-bowling_throw_index3])   # Braille 3               
                waypoint_pub.publish(target_pose)
                strict_pub.publish(Bool(True))
                arrived = False
                throw_ball = False
            else:
                if arrived:
                    if arrived and not throw_ball:
                        rospy.loginfo("Arrived. Start to throw bowling ball 3")
                        # Implement bowling ball throwing code here
                        arm_action = ArmAction.THROW_BOWLING3
                        arm_last_action_time = rospy.get_time()
                        throw_ball = True
                    elif arrived and throw_ball:
                        if arm_finish_state:
                            print(arm_read)
                            rospy.loginfo("Stage 16 done")
                            stage = 17
                        elif rospy.get_time() - arm_last_action_time > BOWLING_BALL_THROW_TIMEOUT:
                            print(rospy.get_time())
                            print(arm_last_action_time)
                            rospy.loginfo("Stage 16 done due to timeout")           
                            stage = 17

        # 17: Done
        if stage == 17:
            if prev_stage == stage-1:
                rospy.loginfo("All finished")
            
        prev_stage = stage  
        basketball_prev_stage = basketball_stage
        bowling_ball_prev_stage = bowling_ball_stage  
        rate.sleep()

def arrived_callback(data):
    global arrived
    arrived = data.data

def get_goal(pose):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = pose['x']
    goal.target_pose.pose.position.y = pose['y']
    goal.target_pose.pose.position.z = 0
    goal.target_pose.pose.orientation.x = 0
    goal.target_pose.pose.orientation.y = 0
    goal.target_pose.pose.orientation.z = pose['qz']
    goal.target_pose.pose.orientation.w = pose['qw']

    return goal

def get_target_pose(pose):
    target_pose = Pose()
    target_pose.position.x = pose['x']
    target_pose.position.y = pose['y']
    target_pose.position.z = 0
    target_pose.orientation.x = 0
    target_pose.orientation.y = 0
    target_pose.orientation.z = pose['qz']
    target_pose.orientation.w = pose['qw']

    return target_pose


if __name__ == '__main__':
    try:
        automation()
    except rospy.ROSInterruptException:
        pass
