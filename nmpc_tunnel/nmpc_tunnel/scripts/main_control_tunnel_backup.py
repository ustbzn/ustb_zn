#!/usr/bin/env python3
# -- coding:UTF-8 --

"""
Author: Nan Sun
Path tracking in tunnel, Backup
"""

import math
import time
import rospy
import numpy as np
import solver_nmpc_mul_shooting as solver
from nav_msgs.msg import Path
from can_msgs.msg import Frame
from std_msgs.msg import Float32MultiArray

gamma = 0
v = 0
de = 0
dn = 0
heading = 0
X = np.zeros([4, 10])


def path_callback(data):
    global X
    X = np.zeros([4, 10])
    for i in range(10):
        X[0, i] = data.poses[i].pose.position.x
        X[1, i] = data.poses[i].pose.position.y
        X[2, i] = data.poses[i].pose.position.z
        X[3, i] = 0


def can_callback(data):
    global gamma
    global v

    # Get articulated angle
    if data.id == 773:
        a = bytearray(data.data)
        can_data = [x for x in a]
        gamma = ((can_data[0] * 256 + can_data[1]) / 100 - 92.5) * math.pi / 180

    # Get vehicle speed
    if data.id == 514:
        a = bytearray(data.data)
        can_data = [x for x in a]
        v = (can_data[1] + can_data[2] * 256) * 2 * 0.519 * math.pi / 10 / 23.25


def control_output():
    global gamma
    global v
    global de
    global dn
    global heading
    global X

    # Starting time
    t0 = time.time()

    # Define parameters
    T = 0.2  # 时间间隔
    n_states = 4  # 状态量个数
    n_controls = 2  # 控制量个数
    Np = 10  # 预测步长
    Nc = 10  # 控制步长
    v_ref = 0.2  # 参考速度

    lr = 1.620  # Length from center of front axle to articulated point
    lf = 1.923  # Length from center of rear axle to articulated point

    # 初始化ROS
    rospy.init_node('main_control')
    rate = rospy.Rate(1 / T)

    rospy.Subscriber('received_messages', Frame, can_callback)
    rospy.Subscriber('ladar_vehicle_coordinate', Path, path_callback)

    ctrl_pub = rospy.Publisher('nmpc_output', Float32MultiArray, queue_size=2)

    # Get first state
    v_last = v
    gamma_last = gamma
    time_last = time.time()

    while not rospy.is_shutdown():
        # 更新状态量+控制量
        xf, yf, thetaf = 0, 0, 0 + 0.25
        time_now = time.time()

        v_now = v
        gamma_now = gamma
        gamma_dot = (gamma_now - gamma_last) / (time_now - time_last)

        # Integration of the matrix
        current_state = np.matrix([[xf], [yf], [thetaf], [-gamma_now]])
        u0 = np.array([[v_now], [-gamma_dot]])

        # 更新参考路径
        next_states = X

        # 准备CasADi矩阵
        # 聚合c_p(控制量+状态量+参考路径)
        c_p = np.concatenate((current_state.reshape(-1, 1), next_states.reshape(-1, 1)))

        # 更新init_control(控制增量)
        init_control = np.concatenate((np.tile(u0.reshape(-1, 1), (Nc, 1)).reshape(-1, 1),
                                       np.tile(current_state, (Np + 1, 1)).reshape(-1, 1)))

        # 求解
        res = solver.nmpc_solver(lf, lr, T, Np, Nc, init_control, c_p)

        # 控制量输出
        estimated_opt = res['x'].full()
        u_all = estimated_opt[:int(n_controls * Nc)].reshape(Nc, n_controls)
        u = u_all[0, :].reshape(-1, 1)

        # 发送控制量至底层控制
        ctrl = [u[0, 0], -u[1, 0]]
        ctrl_pub.publish(Float32MultiArray(data=ctrl))
        print("control variable:", ctrl)

        # 此时刻变量赋值成上一时刻变量
        gamma_last = gamma_now
        time_last = time_now

        rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    try:
        control_output()
    except rospy.ROSInterruptException:
        pass
