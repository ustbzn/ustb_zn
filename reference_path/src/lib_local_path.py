# -*- coding: UTF-8 -*-
import math
import numpy as np
import matplotlib.pyplot as plt
import time
#import scipy.interpolate
import lib_cost_cal


plt.rcParams['font.sans-serif'] = ['SimHei']  # 用来正常显示中文标签
plt.rcParams['axes.unicode_minus'] = False  # 用来正常显示负号
# 设置xtick和ytick的方向：in、out、inout
plt.rcParams['xtick.direction'] = 'in'
plt.rcParams['ytick.direction'] = 'in'

# 车辆参数
Lf = 2
Lr = 2
MAX_ANGLE = np.deg2rad(45)

d_gamma_output = 0

path_target = np.array(((0, 0),
                        (2, 0.4),
                        (4, -0.3),
                        (3.5, 0.0),
                        (5, -0.3)))

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, gamma=0.0, s=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.gamma = gamma
        self.s = s
        # self.c = self.cal_c(self.gamma, d_gamma, Lf, Lr, v)
        self.c = 0.0


def pi_2_pi(angle): # 把角度转换到-pi到pi
    return (angle + math.pi) % (2 * math.pi) - math.pi

def xian_lei_quli(list,X,Y): #计算线与类的距离
   depart = []
   for I_p in range(X.shape[0]):
       for i in range(len(list)):
           for j in list[i]:
               depart.append(math.sqrt((X[I_p] - j[0]) ** 2 + (Y[I_p]- j[1]) ** 2))
               # depart.append(math.sqrt((j[0]) ** 2 + ( j[1]) ** 2))
   return min(depart)


def update(state, v, d_gamma, dt): #状态更新
    state.x = state.x + v * math.cos(state.yaw) * dt
    state.y = state.y + v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + (v * math.sin(state.gamma) / (Lf * math.cos(state.gamma) + Lr) + d_gamma * Lr / (
            Lf * math.cos(state.gamma) + Lr)) * dt
    # state.yaw = pi_2_pi(state.yaw)
    state.gamma = state.gamma + d_gamma * dt
    state.s = state.s + v * dt

    return state


def liang_dian_shi(x1, y1, x2, y2, x): #两点得到直线，再由x得到y
    y = (x - x1) * (y2 - y1) / (x2 - x1) + y1
    return y


def f_d_gamma_linear_4(t_predict, d_gamma_1, d_gamma_2, d_gamma_3, t): #分四段离散化
    t0 = 0
    t1 = t_predict / 6
    t2 = t_predict / 2
    t3 = t_predict * 5 / 6
    # print(t1)
    # print(t2)
    # print(t3)
    # print(d_gamma_1)
    # print(d_gamma_1 / t1)

    d_gamma_0 = d_gamma_output
    #print(d_gamma_0)

    d_gamma = 0
    if t < t1:
        # d_gamma = (d_gamma_1 / t1) * t
        d_gamma = liang_dian_shi(t0, d_gamma_0, t1, d_gamma_1, t)
    elif t < t2:
        d_gamma = liang_dian_shi(t1, d_gamma_1, t2, d_gamma_2, t)
    elif t < t3:
        d_gamma = liang_dian_shi(t2, d_gamma_2, t3, d_gamma_3, t)
    else:
        d_gamma = liang_dian_shi(t3, d_gamma_3, t_predict, 0, t)
    return d_gamma

def f_d_gamma_linear_3(t_predict, d_gamma_1, d_gamma_2, t): #分三段离散化
    t1 = t_predict / 4
    t2 = t_predict * 3 / 4
    # print(t1)
    # print(t2)
    # print(t3)
    # print(d_gamma_1)
    # print(d_gamma_1 / t1)

    d_gamma = 0
    if t < t1:
        d_gamma = (d_gamma_1 / t1) * t
    elif t < t2:
        d_gamma = liang_dian_shi(t1, d_gamma_1, t2, d_gamma_2, t)
    else:
        d_gamma = liang_dian_shi(t2, d_gamma_2, t_predict, 0, t)
    return d_gamma

def local_path(gamma, path_reference, v_online):

    t_predict = 3.0 #预测时间
    d_gamma_1 = np.linspace(-0.3, 0.3, 15)  # 时间离散分段（-0.3到0.3分15段）
    d_gamma_2 = np.linspace(-0.3, 0.3, 15)

    list_all_path = []

    # plt.figure(figsize=(8, 6))
    for i in d_gamma_1:
        for j in d_gamma_2:
            # for k in d_gamma_3:
            state = State(0.0, 0.0, 0.0, gamma, 0.0)
            # state = State(0.0, 0.0, 0.0, np.deg2rad(5), 0.0)
            list = []
            for m in range(int(t_predict / dt)):
                d_gamma = f_d_gamma_linear_3(t_predict, i, j, m * dt)
                # print(d_gamma)
                if state.gamma > MAX_ANGLE or state.gamma < -MAX_ANGLE:
                    state = update(state, v_online, 0, dt)
                else:
                    state = update(state, v_online, d_gamma, dt)

                list.append([state.x, state.y, i, j])
            list_all_path.append(list)

    arr = np.array(list_all_path)
    # np.save('path_all.npy', arr)
    # print(arr.shape)

    # for i in arr:
    #     plt.plot(i[:, 0], i[:, 1], c='grey', lw=0.5)

    list_cost = []
    for i in arr:
        cost = lib_cost_cal.cal_cost(path_reference, i[:, :2], i[0, 2], i[0, 3], 0.0, 0.01, 1.0)
        list_cost.append(cost)
    arr_cost = np.array(list_cost)
    id = arr_cost.argmin()
    # plt.plot(arr[id, :, 0], arr[id, :, 1], c='g', lw=3, label='输出路径')
    return arr[id, :, 0], arr[id, :, 1]

    # plt.plot(path_target[:, 0], path_target[:, 1], c='r', lw=1, label='目标路径')
    # plt.scatter(path_target[:, 0], path_target[:, 1], c='r', zorder=10, label='目标路径点')  
#####////////////////////////////////////////////////////////////////////////////////////////////////////////////
t_predict = 3.0
# 仿真参数
dt = 0.3
# 控制参数
v = 6.0 / 3.6
d_gamma = 0.5
"""
d_gamma_1 = np.linspace(-0.75, 0.75, 5)
d_gamma_2 = np.linspace(-0.6, 0.6, 4)
d_gamma_3 = np.linspace(-0.75,0.75, 5)
"""
d_gamma_1 = np.linspace(-0.75, 0.75, 4)
d_gamma_2 = np.linspace(-0.6, 0.6, 5)
d_gamma_3 = np.linspace(-0.75, 0.75, 4)
gamma = 0
v_online = 5/3.6
list_all_path = []

    # plt.figure(figsize=(8, 6))
for i in d_gamma_1:
    for j in d_gamma_2:
        for k in d_gamma_3:
            state = State(0.0, 0.0, 0.0, gamma, 0.0)
                # state = State(0.0, 0.0, 0.0, np.deg2rad(5), 0.0)
            list = []

            for m in range(int(t_predict / dt)):
                d_gamma = f_d_gamma_linear_4(t_predict, i, j, k, m * dt)
                    # print(d_gamma)
                if state.gamma > MAX_ANGLE or state.gamma < -MAX_ANGLE:
                    state = update(state, v_online, 0, dt)
                else:
                    state = update(state, v_online, d_gamma, dt)

                list.append([state.x, state.y, i, j, k, d_gamma])
            list_all_path.append(list)

arr = np.array(list_all_path)
def local_path_3(gamma, path_reference, v_online,list_pcd,min_poinnt_gap=1.5):
    #start1=time.time()
####################
    #t_predict = 3.0
    #d_gamma_1 = np.linspace(-0.6, 0.6, 5)
    #d_gamma_2 = np.linspace(-0.6, 0.6, 5)
    #d_gamma_3 = np.linspace(-0.6, 0.6, 5)
###########################
    # d_gamma_1 = np.linspace(-0.8, 0.8, 9)
    # d_gamma_2 = np.linspace(-0.5, 0.5, 5)
    # d_gamma_3 = np.linspace(-0.4, 0.4, 5)

    # d_gamma_1 = np.linspace(-0.8, 0.8, 15)
    # d_gamma_2 = np.linspace(-0.5, 0.5, 15)
    # d_gamma_3 = np.linspace(-0.3, 0.3, 15)
    """
    list_all_path = []

    # plt.figure(figsize=(8, 6))
    start2=time.time()
    for i in d_gamma_1:
        for j in d_gamma_2:
            for k in d_gamma_3:
                state = State(0.0, 0.0, 0.0, gamma, 0.0)
                # state = State(0.0, 0.0, 0.0, np.deg2rad(5), 0.0)
                list = []

                for m in range(int(t_predict / dt)):
                    d_gamma = f_d_gamma_linear_4(t_predict, i, j, k, m * dt)
                    # print(d_gamma)
                    if state.gamma > MAX_ANGLE or state.gamma < -MAX_ANGLE:
                        state = update(state, v_online, 0, dt)
                    else:
                        state = update(state, v_online, d_gamma, dt)

                    list.append([state.x, state.y, i, j, k, d_gamma])
                list_all_path.append(list)

    arr = np.array(list_all_path)
    """
    start3 = time.time()


    # #划出所有的触须
    # plt.figure()
    # for i in range(len(arr)):
    #     plt.plot(arr[i, :, 0], arr[i, :, 1], c='g', lw=0.5)
    # plt.axis('equal')
    # plt.xlabel('X/[m]', fontsize=15)
    # plt.ylabel('y/[m]', fontsize=15)
    # # plt.legend('',fontsize=20)
    # plt.xticks(fontsize=12)
    # plt.yticks(fontsize=12)
    # plt.tight_layout()
    # plt.show()


    # np.save('paths', arr)
    # arr = np.load('paths.npy')


    # for i in arr:
    #     plt.plot(i[:, 0], i[:, 1], c='grey', lw=0.5)

    list_cost = []
    for i in arr:
        cost, cost_steady, cost_deviation = lib_cost_cal.cal_cost(path_reference, i[:, :2], i[0, 2], i[0, 3], i[0, 4], 1.0, 1.0, 0.0)
        list_cost.append([cost, cost_steady, cost_deviation])
    arr_cost = np.array(list_cost)
    boom_para = min_poinnt_gap 
    min_gap = boom_para-0.5
    min_gap_sa = [] #save deplace
    count_num = 0
    start4 = time.time()
    ####耗时过长，对代码进行优化，不考虑得不到最优的情况
    """
    while min_gap < boom_para:
        if count_num>(arr.shape[0]):
            min_gap_sa_ar = np.array(min_gap_sa)
            id = min_gap_sa[min_gap_sa_ar [:,0].argmin()][1]
            print("min_gap _sa ************","#############################****************************************",min_gap_sa)
            break
        id = arr_cost[:,0].argmin()
        min_gap = xian_lei_quli(list_pcd,arr[id, :,0],arr[id, :,1])
        print("min_gap  ****************************************************",min_gap)
        min_gap_sa.append([arr_cost[id,0]/min_gap,id])
        arr_cost[id,0] = 1000
        count_num +=1
    """ 
    while min_gap < boom_para:
        id = arr_cost[:,0].argmin()
        min_gap = xian_lei_quli(list_pcd,arr[id, :,0],arr[id, :,1])
        min_gap_sa.append([1/min_gap,id])
        print("min_gap  ****************************************************",min_gap)
        arr_cost[id,0] = 100
        count_num +=1
        if count_num>(arr.shape[0]/4): 
            min_gap_sa_ar = np.array(min_gap_sa)
            id = min_gap_sa[min_gap_sa_ar [:,0].argmin()][1]
            break  
    """
      #优化算法
    while min_gap < boom_para:
        id = arr_cost[:,0].argmin()
        min_gap = xian_lei_quli(list_pcd,arr[id, :,0],arr[id, :,1])
        print("min_gap  ****************************************************",min_gap)
        arr_cost[id,0] = 1000
        count_num +=1
        if count_num>(arr.shape[0]/2):   
            break
    """
    # plt.plot(arr[id, :, 0], arr[id, :, 1], c='g', lw=3, label='输出路径')
    start5= time.time()
    global d_gamma_output
    d_gamma_output = arr[id, 1, 5]
    start6= time.time()
    #print(d_gamma_output)
    print("chuxu_cost_time",start4-start3,start5-start4,start6-start5)
    return arr[id, :, 0], arr[id, :, 1], arr[id, 1, 5], arr_cost[id, 1], arr_cost[id, 2]

    # plt.plot(path_target[:, 0], path_target[:, 1], c='r', lw=1, label='目标路径')
    # plt.scatter(path_target[:, 0], path_target[:, 1], c='r', zorder=10, label='目标路径点')  


def local_path_cirrus(gamma, path_reference, v_online):

    t_predict = 3.0
    d_gamma_1 = np.linspace(-0.5, 0.5, 17)
    d_gamma_2 = np.linspace(-0.2, 0.2, 3)
    d_gamma_3 = np.linspace(-0.2, 0.2, 3)

    # d_gamma_1 = np.linspace(-0.8, 0.8, 9)
    # d_gamma_2 = np.linspace(-0.5, 0.5, 5)
    # d_gamma_3 = np.linspace(-0.4, 0.4, 5)

    # d_gamma_1 = np.linspace(-0.8, 0.8, 15)
    # d_gamma_2 = np.linspace(-0.5, 0.5, 15)
    # d_gamma_3 = np.linspace(-0.3, 0.3, 15)

    list_all_path = []

    # plt.figure(figsize=(8, 6))
    for i in d_gamma_1:
        for j in d_gamma_2:
            for k in d_gamma_3:
                state = State(0.0, 0.0, 0.0, gamma, 0.0)
                # state = State(0.0, 0.0, 0.0, np.deg2rad(5), 0.0)
                list = []

                for m in range(int(t_predict / dt)):
                    d_gamma = f_d_gamma_linear_4(t_predict, i, j, k, m * dt)
                    # print(d_gamma)
                    if state.gamma > MAX_ANGLE or state.gamma < -MAX_ANGLE:
                        state = update(state, v_online, 0, dt)
                    else:
                        state = update(state, v_online, d_gamma, dt)

                    list.append([state.x, state.y, i, j, k, d_gamma])
                list_all_path.append(list)

    arr = np.array(list_all_path)


    # np.save('paths', arr)
    # arr = np.load('paths.npy')


    # for i in arr:
    #     plt.plot(i[:, 0], i[:, 1], c='grey', lw=0.5)

    list_cost = []
    for i in arr:
        cost = lib_cost_cal.cal_cost(path_reference, i[:, :2], i[0, 2], i[0, 3], i[0, 4], 1.0, 1.0, 0.5)
        list_cost.append(cost)
    arr_cost = np.array(list_cost)
    id = arr_cost.argmin()
    # plt.plot(arr[id, :, 0], arr[id, :, 1], c='g', lw=3, label='输出路径')

    global d_gamma_output
    d_gamma_output = arr[id, 1, 5]
    #print(d_gamma_output)
    return arr[id, :, 0], arr[id, :, 1], arr[id, 1, 5]

    # plt.plot(path_target[:, 0], path_target[:, 1], c='r', lw=1, label='目标路径')
    # plt.scatter(path_target[:, 0], path_target[:, 1], c='r', zorder=10, label='目标路径点')  


def local_path_3_offline(gamma, path_reference, v_online):

    # t_predict = 3.0
    # d_gamma_1 = np.linspace(-0.3, 0.3, 25)
    # d_gamma_2 = np.linspace(-0.3, 0.3, 7)
    # d_gamma_3 = np.linspace(-0.3, 0.3, 7)

    # # d_gamma_1 = np.linspace(-0.8, 0.8, 15)
    # # d_gamma_2 = np.linspace(-0.5, 0.5, 15)
    # # d_gamma_3 = np.linspace(-0.3, 0.3, 15)

    # list_all_path = []

    # # plt.figure(figsize=(8, 6))
    # for i in d_gamma_1:
    #     for j in d_gamma_2:
    #         for k in d_gamma_3:
    #             state = State(0.0, 0.0, 0.0, gamma, 0.0)
    #             # state = State(0.0, 0.0, 0.0, np.deg2rad(5), 0.0)
    #             list = []

    #             for m in range(int(t_predict / dt)):
    #                 d_gamma = f_d_gamma_linear_4(t_predict, i, j, k, m * dt)
    #                 # print(d_gamma)
    #                 if state.gamma > MAX_ANGLE or state.gamma < -MAX_ANGLE:
    #                     state = update(state, v_online, 0, dt)
    #                 else:
    #                     state = update(state, v_online, d_gamma, dt)

    #                 list.append([state.x, state.y, i, j, k])
    #             list_all_path.append(list)

    # arr = np.array(list_all_path)


    # np.save('paths', arr)
    gamma_range=[-0.3, 0.3]
    arr = np.load('paths.npy')
    gamma_arr = np.linspace(gamma_range[0], gamma_range[-1], 100)
    min_id = np.argmin(np.abs(gamma_arr - gamma))
    arr = arr[min_id]


    # for i in arr:
    #     plt.plot(i[:, 0], i[:, 1], c='grey', lw=0.5)

    list_cost = []
    for i in arr:
        cost = lib_cost_cal.cal_cost(path_reference, i[:, :2], i[0, 2], i[0, 3], i[0, 4], 0.01, 1.0, 10.0)
        list_cost.append(cost)
    arr_cost = np.array(list_cost)
    id = arr_cost.argmin()
    # plt.plot(arr[id, :, 0], arr[id, :, 1], c='g', lw=3, label='输出路径')
    return arr[id, :, 0], arr[id, :, 1], arr[id, 0, 2]

    # plt.plot(path_target[:, 0], path_target[:, 1], c='r', lw=1, label='目标路径')
    # plt.scatter(path_target[:, 0], path_target[:, 1], c='r', zorder=10, label='目标路径点')  



def test11():
    #p0 = time.process_time()
    dt = 0.3
    # 控制参数
    #v = 8.0 / 3.6
    t_predict = 3.0
    # 仿真参数
    dt = 0.1
    # 控制参数
    v = 6.0 / 3.6
    d_gamma = 0.5
    v = 6.0 / 3.6
    d_gamma = 0.5

    t_predict = 3.0

    # d_gamma_1 = np.linspace(-0.4, 0.4, 5)
    # d_gamma_2 = np.linspace(-0.4, 0.4, 5)
    # d_gamma_3 = np.linspace(-0.4, 0.4, 5)

    d_gamma_1 = np.linspace(-0.75, 0.75, 4)
    d_gamma_2 = np.linspace(-0.6, 0.6, 5)
    d_gamma_3 = np.linspace(-0.75,0.75, 4)

    # d_gamma_1 = np.linspace(-0.2, 0.2, 7)
    # d_gamma_2 = np.linspace(-0.2, 0.2, 7)
    # d_gamma_3 = np.linspace(-0.2, 0.2, 7)

    list_all_path = []

    #plt.figure(figsize=(8, 6))
    for i in d_gamma_1:
        for j in d_gamma_2:
            for k in d_gamma_3:
                state = State(0.0, 0.0, 0.0, np.deg2rad(0), 0.0)
                # state = State(0.0, 0.0, 0.0, np.deg2rad(5), 0.0)
                list = []

                for m in range(int(t_predict / dt)):
                    d_gamma = f_d_gamma_linear_4(t_predict, i, j, k, m * dt)
                    # print(d_gamma)
                    if state.gamma > MAX_ANGLE or state.gamma < -MAX_ANGLE:
                        state = update(state, v, 0, dt)
                    else:
                        state = update(state, v, d_gamma, dt)

                    list.append([state.x, state.y, i, j, k])
                list_all_path.append(list)

    arr = np.array(list_all_path)
    print("arr***************************************************************************************",arr)
    np.save('path_all.npy', arr)
    print(arr.shape)

    for i in arr:
        plt.plot(i[:, 0], i[:, 1], c='grey', lw=0.5)
    #plt.show()
    list_cost = []
    for i in arr:
        cost = lib_cost_cal.cal_cost(path_target, i[:, :2], i[0, 2], i[0, 3], i[0, 4], 1.0, 1.0)
        list_cost.append(cost)
    arr_cost = np.array(list_cost)
    arr_cost1 =  arr_cost[:,0]  
    id = arr_cost1.argmin()
    print("len(arr_cost)",len(arr_cost),arr_cost)
    print("id",id)
    plt.plot(arr[id, :, 0], arr[id, :, 1], c='g', lw=3, label='out_put')
    # plt.plot(path_target[:, 0], path_target[:, 1], c='r', lw=1, label='目标路径')
    plt.scatter(path_target[:, 0], path_target[:, 1], c='r', zorder=10, label='target')
    # plt.legend([l2, l1], ["line 2", "line 1"], loc='upper left')

    """
    plt.axis('equal')
    plt.xlabel(r'X坐标($m$)', fontsize=20)
    plt.ylabel(r'Y坐标($m$)', fontsize=20)
    # plt.xlabel(r'路径长度($m$)', fontsize=20)
    # plt.ylabel(r'曲率($m^{ - 1}$)', fontsize=20)
    # plt.ylim([0, 0.4])
    # plt.xlim([-2, 14])
    # plt.ylim([-10, 10])
    plt.legend(fontsize=20)
    plt.xticks(fontsize=20)
    plt.yticks(fontsize=20)
    #plt.tight_layout()
    """
    plt.show()
    

    #
    # p1 = time.process_time()
    # spend3 = p1 - p0
    # print("process_time()用时：{}s".format(spend3))
    #


if __name__ == '__main__':
    v = 5/3.6
    test11()  # 采样总体效果
