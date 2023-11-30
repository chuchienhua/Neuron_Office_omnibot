#!/usr/bin/env python3
import rospy 
import serial
from time import sleep
import sys


def armmsg():
    rospy.init_node('armmsg')
    ser_arm = serial.Serial(port='/dev/ttyACM2', baudrate=115200, timeout=1)
    ser_arm.isOpen()
    global ctrlmsg

    try:
        while True:
            # 接收用戶的輸入值並轉成小寫
            choice = input('按1開燈、按2關燈、按e關閉程式  ').lower()

            if choice == '1':
                ctrlmsg = b'1,0\n'
                ser_arm.write(ctrlmsg)  # 訊息必須是位元組類型
                sleep(0.5)              # 暫停0.5秒，再執行底下接收回應訊息的迴圈
                print(ctrlmsg)
            elif choice == '2':
                ctrlmsg = b'2,0\n'
                ser_arm.write(ctrlmsg)
                sleep(0.5)
                print(ctrlmsg)
            elif choice == 'e':
                ser_arm.close()
                sys.exit()
            else:
                print('指令錯誤…')

        # while ser.in_waiting:
        #     mcu_feedback = ser.readline().decode()  # 接收回應訊息並解碼
        #     print('控制板回應：', mcu_feedback)
            
    except KeyboardInterrupt:
        ser_arm.close()
        print('再見！')

if __name__ == '__main__':
    try:
        armmsg()
    except rospy.ROSInterruptException:
        pass