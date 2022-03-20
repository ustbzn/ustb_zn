# -*- coding:utf-8 -*-
import numpy as np
import math
import time
"""
计算触须中每条路径的评价参数
"""


def calc_angle(x1,y1,x2,y2): #计算两点的航向角
    angle=0
    dy= y2-y1
    dx= x2-x1
    if dx==0 and dy>0:
        angle = 0
    if dx==0 and dy<0:
        angle = math.pi
    if dy==0 and dx>0:
        angle = math.pi / 2
    if dy==0 and dx<0:
        angle = 3 * math.pi / 2
    if dx>0 and dy>0:
       angle = math.atan(dx/dy)
    elif dx<0 and dy>0:
       angle = 2*math.pi + math.atan(dx/dy)
    elif dx<0 and dy<0:
       angle = math.pi + math.atan(dx/dy)
    elif dx>0 and dy<0:
       angle = math.pi + math.atan(dx/dy)
    return angle

def cal_path_alternative_y(x, path_alternative): #找出path_alternative中横坐标最接近x的点的纵坐标
    idx = np.abs(x - path_alternative[:, 1]).argmin()
    # print(idx)
    y = path_alternative[idx, 1]
    return y

def cal_alternative_d(i, path_alternative): #i点与path_alternative线上的最近距离
    d_arr = np.sqrt(np.square(i[0] - path_alternative[:,0]) + np.square(i[1] - path_alternative[:,1]))
    min_d = d_arr.min()
    return min_d 



# def cal_cost_steady(d_gamma_1, d_gamma_2, d_gamma_3):
#     arr = np.array(((d_gamma_1 - 0), (d_gamma_2 - d_gamma_1), (d_gamma_3 - d_gamma_2), (0 - d_gamma_3)))
#     arr = np.abs(arr)
#     cost = np.sum(arr)
#     return cost



def cal_curverture(x1, y1, x2, y2, x3, y3): # 求曲率
    # int x1, y1, x2, y2, x3, y3;
    # int a, b, c, g, e, f;
    # x1 = px1.x;
    # y1 = px1.y;
    # x2 = px2.x;
    # y2 = px2.y;
    # x3 = px3.x;
    # y3 = px3.y;
    if (y2-y1) / (x2-x1) == (y3-y2) / (x3-x2):
        K = 0
    else: 
        e = 2 * (x2 - x1);
        f = 2 * (y2 - y1);
        g = x2*x2 - x1*x1 + y2*y2 - y1*y1;
        a = 2 * (x3 - x2);
        b = 2 * (y3 - y2);
        c = x3*x3 - x2*x2 + y3*y3 - y2*y2;
        X = (g*b - c*f) / (e*b - a*f);
        Y = (a*g - c*e) / (a*f - b*e);
        R = np.sqrt((X-x1)*(X-x1)+(Y-y1)*(Y-y1));
        K = 1/R
    return K

def cal_cost_steady(path_alternative): #平稳性评价参数
    # i = len(path_alternative)
    # print(path_alternative)
    K = 0
    for i in range(len(path_alternative)):
        if i == 0:
            continue
        if i == len(path_alternative)-1:
            break
        Ki = cal_curverture(path_alternative[i-1,0], path_alternative[i-1,1], path_alternative[i,0], path_alternative[i,1], path_alternative[i+1,0], path_alternative[i+1,1])
        K = K + Ki
    cost = K/(len(path_alternative)-2)
    #print('cost_steady:')
    #print(cost)
    return cost


def cal_cost_deviation(path_target, path_alternative): #安全评价系数
    # print(path_target)
    # print(path_alternative)
    cost = 0.0
    for i in path_target:
        # y = cal_path_alternative_y(i, path_alternative)
        y = cal_alternative_d(i, path_alternative)
        # print('----')
        # print(i)
        # print(i[0])
        # dif = np.abs(float(y) - float(i[1]))
        dif = y
        cost = cost + dif
        cost = cost/len(path_target)



    return cost

def cal_cost_yaw(path_target, path_alternative): #终点航向的差值
    # yaw_target = (path_target[-1, 1] - path_target[-2, 1]) / (path_target[-1, 0] - path_target[-2, 0])
    yaw_target = calc_angle(path_target[-2, 0], path_target[-2, 1], path_target[-1, 0], path_target[-1, 1])
    # print(path_target)
    # print(yaw_target)
    # yaw_alternative = (path_alternative[-1, 1] - path_alternative[-2, 1]) / (path_alternative[-1, 0] - path_alternative[-2, 0])
    yaw_alternative = calc_angle(path_alternative[-2, 0], path_alternative[-2, 1], path_alternative[-1, 0], path_alternative[-1, 1])
    # print(yaw_alternative)
    cost = np.abs(yaw_alternative - yaw_target)
    return cost


def cal_cost(path_target, path_alternative, d_gamma_1, d_gamma_2, d_gamma_3, k_s=2.0, k_d=1.0, k_y=1.0): #触须算法的总评价参数
    # cost_steady = cal_cost_steady(d_gamma_1, d_gamma_2, d_gamma_3)
    cost_steady = cal_cost_steady(path_alternative)
    cost_deviation = cal_cost_deviation(path_target, path_alternative)
    cost_yaw = cal_cost_yaw(path_target, path_alternative)
    cost = k_s * cost_steady + k_d * cost_deviation + k_y * cost_yaw
    return cost, cost_steady, cost_deviation+cost_yaw


if __name__ == '__main__':
    #print('cost_cal')
    path_target = np.array(((0, 0), (10, 1), (20, 0)))
    path_alternative = np.array((0, 0))

