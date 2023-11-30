#!/usr/bin/env python3
import rospy    
import serial
import threading
from enum import Enum
 
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

ctrl_state = CtrlState.IDLE
action = Action.NONE

# Communicate with TDK control box
def tdk_ctrl_box_com():
    global ctrl_state
    global action
    ser = serial.Serial(port='/dev/tdk', baudrate=115200,timeout=2,rtscts=True,dsrdtr=True)
    ser.isOpen()
    while True:
        ser.write(ctrl_state.value)

        if ser.in_waiting:
            res = ser.readline()
            action = Action(res)            
        rospy.sleep(0.1)

def automation():    
    global ctrl_state
    global action

    # Start TDK control box communication
    t = threading.Thread(target=tdk_ctrl_box_com)
    t.start()

    while True:                
        if action == Action.START_A:
            ctrl_state = CtrlState.RUNNING_A
        elif action == Action.START_B:
            ctrl_state = CtrlState.RUNNING_B
        elif action == Action.STOP:
            ctrl_state = CtrlState.IDLE



if __name__ == '__main__':
    try:
        automation()
    except rospy.ROSInterruptException:
        pass
