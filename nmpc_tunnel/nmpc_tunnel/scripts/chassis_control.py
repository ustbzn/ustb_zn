#!/usr/bin/env python3
# -- coding:UTF-8 --


import time
import rospy
import select
import sys
import termios
import tty
from can_msgs.msg import Frame
from std_msgs.msg import Float32MultiArray

ctrl = [0, 0]


def gear_select(gear_value):
    # Neutral
    gear1 = 0
    gear2 = 0
    gear3 = 0
    gear4 = 0
    if gear_value == 1:  # Drive 1st
        gear2 = 1
        gear3 = 1
        gear4 = 1
    elif gear_value == 2:  # Drive 2nd
        gear2 = 1
        gear3 = 1
    elif gear_value == 3:  # Drive 3rd
        gear3 = 1
    elif gear_value == -1:  # Reverse 1st
        gear1 = 1
        gear2 = 1
        gear4 = 1
    elif gear_value == -2:  # Reverse 2nd
        gear1 = 1
        gear4 = 1
    elif gear_value == -3:  # Reverse 3rd
        gear1 = 1
    return gear1, gear2, gear3, gear4


def data_transfer_can_517(id_cmd, ROS_VCU_Pctthrottlevalue1, ROS_VCU_SteerFreq, ROS_VCU_Pctthrottlevalue1Freq, count):
    if ROS_VCU_SteerFreq > 2000:
        ROS_VCU_SteerFreq = 2000

    data0 = ROS_VCU_Pctthrottlevalue1 % 256
    data1 = ROS_VCU_Pctthrottlevalue1 // 256
    data2 = ROS_VCU_SteerFreq % 256
    data3 = ROS_VCU_SteerFreq // 256 + (ROS_VCU_Pctthrottlevalue1Freq % 32) * 8
    data4 = ROS_VCU_Pctthrottlevalue1Freq // 32

    data_array = bytearray([data0, data1, data2, data3, data4, 0, 0, count])

    cmd2 = Frame()
    cmd2.header.stamp = rospy.Time.now()
    cmd2.header.frame_id = ''
    cmd2.id = id_cmd
    cmd2.is_rtr = False
    cmd2.is_extended = False
    cmd2.is_error = False
    cmd2.dlc = 0x8
    cmd2.data = data_array

    return cmd2


def data_transfer_can_516(id_cmd, ROS_VCU_flgParking, ROS_VCU_flgGear1, ROS_VCU_flgGear2,
                          ROS_VCU_flgGear3, ROS_VCU_flgGear4, ROS_VCU_flgSteeringValve,
                          ROS_VCU_flgSteerDirection, ROS_VCU_flgSteerEnable, ROS_VCU_PctSteer,
                          ROS_VCU_PctUp, ROS_VCU_PctDown, ROS_VCU_PctBrakingF, ROS_VCU_PctBrakingR,
                          ROS_VCU_Pctthrottlevalue, count, ROS_VCU_flgHorn):
    data0 = ROS_VCU_flgParking * pow(2, 0) + \
            ROS_VCU_flgGear1 * pow(2, 1) + ROS_VCU_flgGear2 * pow(2, 2) + \
            ROS_VCU_flgGear3 * pow(2, 3) + ROS_VCU_flgGear4 * pow(2, 4) + \
            ROS_VCU_flgSteeringValve * pow(2, 5) + ROS_VCU_flgSteerDirection * pow(2, 6) + \
            ROS_VCU_flgSteerEnable * pow(2, 7)
    data1 = ROS_VCU_PctSteer + ROS_VCU_flgHorn * pow(2, 7)
    data_array = bytearray([data0, data1, ROS_VCU_PctUp, ROS_VCU_PctDown,
                            ROS_VCU_PctBrakingF, ROS_VCU_PctBrakingR, ROS_VCU_Pctthrottlevalue, count])

    cmd = Frame()
    cmd.header.stamp = rospy.Time.now()
    cmd.header.frame_id = ''
    cmd.id = id_cmd
    cmd.is_rtr = False
    cmd.is_extended = False
    cmd.is_error = False
    cmd.dlc = 0x8
    cmd.data = data_array

    return cmd


def control_callback(data):
    global ctrl
    ctrl = data.data
    print(ctrl)


def chassis_control(argv):
    global ctrl

    rospy.init_node('test_dynamic', anonymous=True)
    rospy.Subscriber('nmpc_output', Float32MultiArray, control_callback)
    cmd_pub = rospy.Publisher('sent_messages', Frame, queue_size=2)

    old_attr = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    T = 0.01
    rate = rospy.Rate(1 / T)

    id_cmd = 0x204
    id_cmd2 = 0x205

    count = 0

    ROS_VCU_flgParking = 0  # 驻车控制指令
    ROS_VCU_flgSteeringValve = 0  # 转向阀控制指令 (0 zhuanxiang 1 jusheng)
    # ROS_VCU_flgSteerEnable = 0  # 转向使能控制指令 (o keyizhuanxiang 1 bunengzhuanxiang)
    ROS_VCU_PctUp = 0  # 举升开度控制指令
    ROS_VCU_PctDown = 0  # 下降开度控制指令
    ROS_VCU_PctBrakingF = 0  # 制动开度控制指令
    ROS_VCU_PctBrakingR = 0  # 行车开度控制指令
    ROS_VCU_Pctthrottlevalue = 0  # has been abandoned
    ROS_VCU_flgHorn = 0  # laba

    ROS_VCU_flgSteerDirection = 0
    t1 = time.time()
    while not rospy.is_shutdown():
        

        gear_value = 1
        throttle_value = 60
        ROS_VCU_Pctthrottlevalue1Freq = 2000
        steering_value = 50
        steer = ctrl[1]

        ROS_VCU_Pctthrottlevalue1 = int(throttle_value * 100)

        # if  -0.005 < steer < 0.005:
          #  ROS_VCU_SteerFreq = 0
          #  ROS_VCU_PctSteer = 0
          #  ROS_VCU_flgSteerEnable = 1
        # else:
          # ROS_VCU_SteerFreq = abs(int(17461 * steer - 14.704))
          # ROS_VCU_PctSteer = 50
          # ROS_VCU_flgSteerEnable = 0
        ROS_VCU_SteerFreq = abs(int(14000 * steer))
        ROS_VCU_PctSteer = 50
        ROS_VCU_flgSteerEnable = 0
        # 0 left 1 right
        if steer > 0:
            ROS_VCU_flgSteerDirection = 0
        elif steer < 0:
            ROS_VCU_flgSteerDirection = 1

        ROS_VCU_flgGear1, ROS_VCU_flgGear2, ROS_VCU_flgGear3, ROS_VCU_flgGear4 = gear_select(gear_value)  # 档位控制指令
        # print(ROS_VCU_SteerFreq)
        cmd_can_516 = data_transfer_can_516(id_cmd, ROS_VCU_flgParking, ROS_VCU_flgGear1, ROS_VCU_flgGear2,
                                            ROS_VCU_flgGear3, ROS_VCU_flgGear4, ROS_VCU_flgSteeringValve,
                                            ROS_VCU_flgSteerDirection, ROS_VCU_flgSteerEnable, ROS_VCU_PctSteer,
                                            ROS_VCU_PctUp, ROS_VCU_PctDown, ROS_VCU_PctBrakingF, ROS_VCU_PctBrakingR,
                                            ROS_VCU_Pctthrottlevalue, count, ROS_VCU_flgHorn)
        cmd_can_517 = data_transfer_can_517(id_cmd2, ROS_VCU_Pctthrottlevalue1, ROS_VCU_SteerFreq,
                                            ROS_VCU_Pctthrottlevalue1Freq,
                                            count)

        cmd_pub.publish(cmd_can_516)
        cmd_pub.publish(cmd_can_517)

        if count < 15:
            count = count + 1
        else:
            count = 0

        t2 = time.time()
        if t2 - t1 > 120:
            print('The test process has exceeded 30 seconds')
            ROS_VCU_Pctthrottlevalue1Freq = 0
            ROS_VCU_Pctthrottlevalue1 = 0
            ROS_VCU_PctSteer = 0
            ROS_VCU_SteerFreq = 0
            ROS_VCU_flgSteerEnable = 1

            ROS_VCU_flgGear1, ROS_VCU_flgGear2, ROS_VCU_flgGear3, ROS_VCU_flgGear4 = gear_select(0)
            cmd_can_516 = data_transfer_can_516(id_cmd, ROS_VCU_flgParking, ROS_VCU_flgGear1, ROS_VCU_flgGear2,
                                                ROS_VCU_flgGear3, ROS_VCU_flgGear4, ROS_VCU_flgSteeringValve,
                                                ROS_VCU_flgSteerDirection, ROS_VCU_flgSteerEnable, ROS_VCU_PctSteer,
                                                ROS_VCU_PctUp, ROS_VCU_PctDown, ROS_VCU_PctBrakingF,
                                                ROS_VCU_PctBrakingR, ROS_VCU_Pctthrottlevalue, count, ROS_VCU_flgHorn)
            cmd_can_517 = data_transfer_can_517(id_cmd2, ROS_VCU_Pctthrottlevalue1, ROS_VCU_SteerFreq,
                                                ROS_VCU_Pctthrottlevalue1Freq,
                                                count)

            cmd_pub.publish(cmd_can_516)
            cmd_pub.publish(cmd_can_517)
            print('The stop command has been sent')
            break

        if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:  # 设置超时参数为0，防止阻塞
            print('The test process has been killed by keyboard')
            ROS_VCU_Pctthrottlevalue1Freq = 0
            ROS_VCU_Pctthrottlevalue1 = 0
            ROS_VCU_PctSteer = 0
            ROS_VCU_SteerFreq = 0
            ROS_VCU_flgSteerEnable = 1

            ROS_VCU_flgGear1, ROS_VCU_flgGear2, ROS_VCU_flgGear3, ROS_VCU_flgGear4 = gear_select(0)
            cmd_can_516 = data_transfer_can_516(id_cmd, ROS_VCU_flgParking, ROS_VCU_flgGear1, ROS_VCU_flgGear2,
                                                ROS_VCU_flgGear3, ROS_VCU_flgGear4, ROS_VCU_flgSteeringValve,
                                                ROS_VCU_flgSteerDirection, ROS_VCU_flgSteerEnable, ROS_VCU_PctSteer,
                                                ROS_VCU_PctUp, ROS_VCU_PctDown, ROS_VCU_PctBrakingF,
                                                ROS_VCU_PctBrakingR, ROS_VCU_Pctthrottlevalue, count, ROS_VCU_flgHorn)
            cmd_can_517 = data_transfer_can_517(id_cmd2, ROS_VCU_Pctthrottlevalue1, ROS_VCU_SteerFreq,
                                                ROS_VCU_Pctthrottlevalue1Freq,
                                                count)

            cmd_pub.publish(cmd_can_516)
            cmd_pub.publish(cmd_can_517)
            print('The stop command has been sent')
            break

        rate.sleep()

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)  # 设置为原先的标准模式
    print('PLease press Ctrl+C to exit')

    rospy.spin()


if __name__ == '__main__':
    argv = rospy.myargv()
    try:
        chassis_control(argv)
    except rospy.ROSInterruptException:
        pass
