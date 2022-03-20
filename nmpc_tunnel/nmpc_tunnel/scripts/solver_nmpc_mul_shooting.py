#!/usr/bin/env python3
# -- coding:UTF-8 --

import casadi as ca
import numpy as np


def nmpc_solver(lf, lr, T, Np, Nc, init_control, c_p):
    """
    铰接车非线性模型预测控制器（Nonlinear Model Predictive Controller - NMPC）
    Version of CasADi_Multiple_shooting
    """

    # ！参数定义
    # lf = 1.620  # 铰接车前车体车桥中心点到铰接点的距离
    # lr = 1.923  # 铰接车后车体车桥中心点到铰接点的距离

    # ！描述车辆状态方程
    # ！！定义状态变量
    xf = ca.SX.sym('xf')  # x轴位置
    yf = ca.SX.sym('yf')  # y轴位置
    thetaf = ca.SX.sym('thetaf')  # 航向角
    gamma = ca.SX.sym('gamma')  # 铰接角
    states = ca.vertcat(xf, yf, thetaf, gamma)  # 构建LHD的状态量矩阵，此处需要以(Nx * 1)的格式呈现
    n_states = states.size()[0]  # 获取状态量的尺寸

    # ！！定义控制变量
    vf = ca.SX.sym('vf')  # 纵向速度
    gamma_dot = ca.SX.sym('gamma_dot')  # 铰接角速度
    controls = ca.vertcat(vf, gamma_dot)  # 构建LHD的控制量矩阵，此处需要以（Nc * 1）的格式呈现
    n_controls = controls.size()[0]  # 获取控制量的尺寸

    # ！！构建运动学模型
    rhs = ca.vertcat(vf * ca.cos(thetaf),
                     vf * ca.sin(thetaf),
                     (vf * ca.sin(gamma) / (lr + lf * ca.cos(gamma))) + (gamma_dot * lr / (lr + lf * ca.cos(gamma))),
                     gamma_dot)

    # ！！利用CasADi构建一个函数来代表运动学模型，测出类似与python的
    # def f(states, controls):
    #     return rhs
    f = ca.Function('f', [states, controls], [rhs], ['input_state', 'control_input'], ['rhs'])

    # ！构建NMPC
    # ！！定义相关矩阵
    U = ca.SX.sym('U', n_controls, Nc)  # Nc步的控制输出
    X = ca.SX.sym('X', n_states, Np + 1)  # Np步的预测输出
    X_now = ca.SX.sym('X_now', n_states, 1)  # 铰接车此时刻位置
    X_ref = ca.SX.sym('X_ref', n_states, Np)  # 铰接车下一时刻位置

    # ！！定义Q,R矩阵    
    Q = np.array([[0.1, 0.0, 0.0, 0.0],
                  [0.0, 0.1, 0.0, 0.0],
                  [0.0, 0.0, 0.1, 0.0],
                  [0.0, 0.0, 0.0, 0.0]])  # (Nx * Nx)
    R = np.array([[0.0001, 0.0],
                  [0.0, 0.0001]])  # (Nc * Nc)

    # ！！定义目标函数
    obj = 0  # 初始化目标函数
    g = []  # 创建存储X差值的矩阵
    g.append(X[:, 0] - X_now[:, 0])

    # ！！计算Np时域内的铰接车预测位置
    for i in range(Np):
        if i <= Nc - 1:
            obj = obj + ca.mtimes([(X[:, i] - X_ref[:, i]).T, Q, (X[:, i] - X_ref[:, i])]) \
                  + ca.mtimes([U[:, i].T, R, U[:, i]])
            x_next_ = f(X[:, i], U[:, i]) * T + X[:, i]
            g.append(X[:, i + 1] - x_next_)  # 对状态量进行差值（此处为何Single_shooting的唯一区别）
        else:  # 超出控制时域Nc后，控制量都以第Nc步的值计算
            obj = obj + ca.mtimes([(X[:, i] - X_ref[:, i]).T, Q, (X[:, i] - X_ref[:, i])]) \
                  + ca.mtimes([U[:, Nc - 1].T, R, U[:, Nc - 1]])
            x_next_ = f(X[:, i], U[:, Nc - 1]) * T + X[:, i]
            g.append(X[:, i + 1] - x_next_)  # 对状态量进行差值

    # ！！定义求解器的变量（包含状态量的差值）
    opt_variables = ca.vertcat(ca.reshape(U, -1, 1), ca.reshape(X, -1, 1))
    opt_params = ca.vertcat(ca.reshape(X_now, -1, 1), ca.reshape(X_ref, -1, 1))

    # ！定义NLP问题，‘f’为目标函数，'x'为需要寻找的优化结果（此处未控制量），'p'为系统参数（此刻装状态量+参考路径），'g'为状态量约束
    nlp_prob = {'f': obj, 'x': opt_variables, 'p': opt_params, 'g': ca.vertcat(*g)}

    # ！！求解器参数设置
    opts_setting = {'ipopt.max_iter': 1000000, 'ipopt.print_level': 0, 'print_time': 0,
                    'ipopt.acceptable_tol': 1e-8, 'ipopt.acceptable_obj_change_tol': 1e-6}

    # ！！定义求解器
    solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts_setting)

    # ！！定义约束
    lbg = 0.0
    ubg = 0.0
    lbx = []
    ubx = []

    for _ in range(Nc):
        lbx.append(0)
        lbx.append(-0.14)
        ubx.append(2)
        ubx.append(0.14)
    for _ in range(Np + 1):  # note that this is different with the method using structure
        lbx.append(-np.inf)
        lbx.append(-np.inf)
        lbx.append(-np.inf)
        lbx.append(-0.75)
        ubx.append(np.inf)
        ubx.append(np.inf)
        ubx.append(np.inf)
        ubx.append(0.75)

    res = solver(x0=init_control, p=c_p, lbg=lbg, lbx=lbx, ubg=ubg, ubx=ubx)

    return res
