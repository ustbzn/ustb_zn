#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import time
import numpy as np
import matplotlib.pyplot as plt
from sklearn import svm
import math
import lib_JuLei
#import numpy as np

def get_crossing_point(k1, b1, k2, b2):#求两条直线的交点
    start = time.time()
    x = (b2 - b1) / (k1 - k2)
    y = (k2 * b1 - k1 * b2) / (k2 - k1)
    start1=time.time()
    print("jiaodiang",start1-start)
    return x, y

'''
对得到的点分类，并且由一类中的两点得到对应的曲线
'''
def run_svm(X, y, data_range, draw=False): #寻找两个聚类间的分割平面（一维平面）
    clf = svm.SVC(kernel='linear')
    clf.fit(X, y) #X为数据，Y为标签

    W = clf.coef_
    B = clf.intercept_
    # print('w')
    # print(w)
    # print(b)
    k = 0.0
    b = 0.0
    if W[0, 1] == 0:
        yy = np.linspace(0, data_range)
        xx = np.empty_like(yy)
        # print(-b / w[0, 0])
        xx.fill(float(-B[0] / W[0, 0]))
        k = -1
        b = -1
    else:
        k = -W[0][0] / W[0][1]
        b = -B[0] / W[0][1]

        xx = np.linspace(0, data_range)
        yy = k * xx + b

    if draw:
        plt.scatter(X[:, 0], X[:, 1], marker='.', s=5)
        plt.scatter(0, 0, marker='>', s=50, c='g')
        plt.xlim(-10, 30)
        plt.ylim(-15, 15)

    return k, b
def K_leastsquare(qulei_class):
    t1 = 0.000
    t2 = 0.000
    t3 = 0.000
    t4 = 0.000
    mo_pre = 0.000 #模型预测错误
    me_pre = 0.000 #使用平均值产生的错误
    for i in qulei_class:
        t1 += i[0]*i[0]
        t2 += i[0]
        t3 += i[0]*i[1]
        t4 += i[1]
    a = (t3*len(qulei_class)-t2*t4)/(t1*len(qulei_class)-t2*t2)
    b = (t1*t4-t2*t3)/(t1*len(qulei_class)-t2*t2)
    for i in qulei_class:
        mo_pre += pow((i[1]-a*i[0]-b),2)
        me_pre += pow((i[1]-t4/len(qulei_class)),2)
    r = pow(1 - mo_pre/me_pre, 0.5)
    return a, b, r
def get_lei_k(qulei_class):
    t1 = 0.000
    t2 = 0.000
    t3 = 0.000
    t4 = 0.000
    for i in qulei_class:
        t1 += i[0]*i[0]
        t2 += i[0]
        t3 += i[0]*i[1]
        t4 += i[1]
    if abs((t1*len(qulei_class)-t2*t2)) > 0.000001:
        a = (t3*len(qulei_class)-t2*t4)/(t1*len(qulei_class)-t2*t2)
    else:
        a = 100
    return a

def center_line(list1,list2,data_range=20, num_of_lines=4):
        start1 = time.time()   
        arr_l = np.array(list1)
        arr_r = np.array(list2)
        length_l = math.sqrt((arr_l[0, 0] - arr_l[-1, 0]) ** 2 + (arr_l[0, 1] - arr_l[-1, 1]) ** 2)  # 左边点的长度
        length_r = math.sqrt((arr_r[0, 0] - arr_r[-1, 0]) ** 2 + (arr_r[0, 1] - arr_r[-1, 1]) ** 2)  # 右边点长度
        range_divide = min(length_l, length_r) / num_of_lines  # 将所有点分成三段，求中心线 // 商取整（分段距离）
        lines = []
        points = []
        start2 = time.time() 
        if len(list1) > 6 and len(list2) > 6:
                # num_l = int(len(list[0]) / num_of_lines) #左边点分成三段
                # num_r = int(len(list[1]) / num_of_lines) #右边点分成三段

                for i in range(num_of_lines):
                    start22= time.time()
                    X_0 = []
                    X_1 = []

                    l_base = list1[0]  # 左边聚类的第一个点（x0，y0）
                    r_base = list2[-1]  # 右边聚类的第一个点(原来为为list2[-1]
                    for j in list1:
                        if (range_divide * i) < np.sqrt((j[0] - l_base[0]) ** 2 + (j[1] - l_base[1]) ** 2) < (
                                range_divide * (i + 1)):  # 判断该点是否在分段距离内，如果在加入X0，不行去除。
                            X_0.append([j[0], j[1]])

                    for j in list2:
                        if (range_divide * i) < np.sqrt((j[0] - r_base[0]) ** 2 + (j[1] - r_base[1]) ** 2) < (
                                range_divide * (i + 1)):  # 判断该点是否在分段距离内，如果在加入X0，不行去除。
                            X_1.append([j[0], j[1]])
                    if num_of_lines == 1:
                        X_0=list1
                        X_1 = list2
                    # 设置标签指出两组数据为所要处理的，使程序能够识别出这是两组数据
                    y_0 = np.zeros([len(X_0)], dtype='int')  # 人为设置标签为0
                    y_1 = np.ones([len(X_1)], dtype='int')  # 设置标签为1
                    start3 = time.time() 
                    #print("x_0,x_1", X_0,X_1)

                    if len(X_0) > 1 and len(X_1):  # 保证数据不为零
                        X = np.r_[X_0, X_1]
                        y = np.r_[y_0, y_1]  # 将数据加入X，标签加入Y
                        k, b = run_svm(X, y, data_range, False)  # SVM求得中心线
                        lines.append([k, b])  # 加入line
                        points.append([X_0[0][0], X_0[0][1], X_1[0][0], X_1[0][1]])  # 各取左右聚类满足分段距离要求的第一个点
                        start4 = time.time() 
                        print("center_line-time_svm",start4-start3,start3-start22,start4-start22)
        #print('center_line-__k,b:', lines)
        start33=time.time()
        path_point = []
        for i in range(len(lines)):
            st1= time.time()
            if abs(points[i][2] - points[i][0]) < 1e-6:
                points[i][2] = points[i][0]+1e-6
            st2= time.time()
            k = (points[i][3] - points[i][1]) / (points[i][2] - points[i][0])
            b = points[i][1] - k * points[i][0]  # 求得由各段左右聚类的第一个点得到直线
            st3= time.time()
            x, y = get_crossing_point(lines[i][0], lines[i][1], k, b)  # 得到中心线和分段聚类第一点得到的直线的交点（有三点可能只有一点）
            st4 = time.time()
            st5 = time.time()
            path_point.append([x, y])
            st6=time.time()
            print("zhixiang yu zhixiang jiao dian ",st2-st1,st3-st2,st4-st3,st5-st4,st6-st5,st6-st1)
        # print("points:",points)
        # print("len(lines_1):", len(lines))
        # print("lines_1:",lines)
        #print("path_point:", path_point)
        start44= time.time() 
        if num_of_lines > 1:
            start5= time.time()
            print('num=n,center_line',start2-start1,start33-start2,start44-start33,start5-start44,start5-start1)
            return path_point
        else:
            start5= time.time()
            print('num=0,center_line',start2-start1,start33-start2,start44-start33,start5-start44,start5-start1)
            return lines

def lei_dian(list_in, point_num=10):#从聚类上均匀取点      #######################没有使用
    point_r = np.array(list_in)
    length = math.sqrt((point_r[0, 0] - point_r[-1, 0]) ** 2 + (point_r[0, 1] - point_r[-1, 1]) ** 2)  # 点的长度
    range_divide = length // point_num  # 将所有点分成三段，求中心线 // 商取整（分段距离）
    point = [list_in[0]]
    l_base= list_in[-1]
    for i_1 in range(point_num):
        x_z = 0
        y_z = 0
        num_c = 0
        for j in list_in:
            if (range_divide * i_1) < np.sqrt((j[0] - l_base[0]) ** 2 + (j[1] - l_base[1]) ** 2) < (
                    range_divide * (i_1 + 1)):  # 判断该点是否在分段距离内，如果在加入X0，计算平均。
                x_z += j[0]
                y_z += j[1]
                num_c += 1
            if abs(x_z) >0.2 and abs(y_z) > 0.2:
                point.append([j[0]/num_c, j[1]/num_c])
    point.append(list_in[-1])
    return point


def lei_center(dd_l,dd_k,list0, list1, list2,control=1,dvi_num=7):  #根据某一类list0-rigeht，list2-left，首先得到离车最近点，平移一个车宽
    start1 = time.time()
    depart = []
    select_pot = []
    list0_s = []
    list2_s = []
    if control != 0:
        if control == 1:
                for j in list0:
                    depart.append(math.sqrt((0 - j[0]) ** 2 + (0 - j[1]) ** 2))
                select = depart.index(min(depart))
                select_max =depart.index(max(depart))
                #print("result1:", select, list0[select])
                if abs(list0[select][0]) < 0.001:
                    list0[select][0] = 0.001
                k_select = list0[select][1]/list0[select][0]
                if abs(k_select) < 0.001:
                    k_select = 0.001
                #dx_k= dd_k/(1 + pow((-1/k_select), 2))
                #dy_k= (-1/k_select)*dx_k
                dx_k=0
                dy_k=0
                dx_l=(k_select/abs(k_select))*(dd_l/(1 + pow((k_select), 2)))
                dy_l = (dd_l/(1 + pow((1/k_select), 2)))
                #print("dx_l",dx_l,"dx_k",dx_k)
                for j in range(select_max-select):
                    list0_s.append(list0[j+select])
                space_num = int(len(list0_s) / dvi_num)
                if space_num<1:
                    space_num = 1
                for i in range(dvi_num):
                    s_num = i * space_num
                    if (s_num+2)>len(list0_s):
                        break
                    pot = list0_s[ s_num]
                    [s_num]
                    if s_num < len(list0_s):
                        select_pot.append([(pot[0]+dx_k+dx_l), (pot[1] + dy_k+dy_l)])
        elif control == 2:
                for j in list2:
                    depart.append(math.sqrt((0 - j[0]) ** 2 + (0 - j[1]) ** 2))
                select = depart.index(min(depart))
                select_max =depart.index(max(depart))
                #print("result2:", select, list2[select])
                if abs(list2[select][0]) < 0.001:
                    list2[select][0] = 0.001
                k_select = list2[select][1] / list2[select][0]
                if abs(k_select) < 0.001:
                    k_select = 0.001
                #dx_k = dd_k / (1 + pow((-1 / k_select), 2))
                #dy_k = (-1 / k_select) *  dx_k 
                dx_k=0
                dy_k=0
                dx_l=(-k_select/abs(k_select))*(dd_l/(1 + pow((k_select), 2)))
                dy_l = -(dd_l/(1 + pow((1/k_select), 2)))
                #print("dx_l",dx_l,"dx_k",dx_k)
                for j in range(select-select_max):
                    list2_s.append(list2[j+select_max])
                list2_s.reverse()
                space_num = int(len(list2_s) / dvi_num)
                if space_num < 1:
                    space_num = 1
                for i in range(dvi_num):
                    s_num = i * space_num
                    if s_num < len(list2_s):
                        if (s_num+1)>len(list2_s):
                            break
                        pot = list2_s[s_num]
                        select_pot.append([(pot[0] + dx_k + dx_l), (pot[1] + dy_k + dy_l)])
        start2 = time.time()
        print("lei_center_control=0",start2-start1)
        return select_pot

    else:
        if len(list0) > len(list2):       
            space_num = int(len(list0) / dvi_num)
            for i in range(dvi_num):
                    s_num = i * space_num
                    for j in list1:
                        pot = list0[s_num]
                        depart.append(math.sqrt((pot[0] - j[0]) ** 2 + (pot[1] - j[1]) ** 2))
                    select = depart.index(min(depart))
                    #print("result3:", select, list1[select])
                    select_pot.append([(pot[0] + list1[select][0]) / 2, (pot[1] + list1[select][1]) / 2])
                    depart = []

        else:
                for i in range(dvi_num):
                    space_num = int(len(list2) / dvi_num)
                    s_num = i * space_num
                    pot = list2[(len(list2) - 1 - s_num)]
                    for j in list1:
                        depart.append(math.sqrt((pot[0] - j[0]) ** 2 + (pot[1] - j[1]) ** 2))
                    select = depart.index(min(depart))
                    #print("result4:", select, list1[select])
                    select_pot.append([(pot[0] + list1[select][0]) / 2, (pot[1] + list1[select][1]) / 2])
                    depart = []
    start2 = time.time()
    print("lei_center_control fei 0",start2-start1)
    return select_pot 
def lei_center2(list0, list1, list2,control=1,dvi_num=7,n_of_line=6):  # 求转弯时的聚类中心,list1为左或右的类，list2为中2间类,对类进行补全，得到中心路径（以一个类为基准补全，之后svm）
    start1 = time.time()
    depart = []
    select_pot = []
    list0_s = []
    list2_s = []
    depart = []
    select_pot = []
    select_dep1 = []
    select_num1=[]
    select_dep2 = []
    select_num2=[]
    list_pu = []
    select_dep_point = []#存贮对应点的距离，对应点索引
    if control != 0:
        if control == 1:
                for j in list0:
                    depart.append(math.sqrt((0 - j[0]) ** 2 + (0 - j[1]) ** 2))
                select = depart.index(min(depart))
                select_max =depart.index(max(depart))
                #print("result1:", select, list0[select])
                depart = [] #初始化
                if len(list0)<25:
                    select = 0 
                    select_max = len(list0)-1
                for j in range(select_max-select):
                    list0_s.append(list0[j+select])
                space_num = int(len(list0_s) / dvi_num)
                for i in range(dvi_num):
                    s_num = i * space_num
                    pot = list0_s[s_num]
                    for j in list1:
                        depart.append(math.sqrt((pot[0] - j[0]) ** 2 + (pot[1] - j[1]) ** 2))
                    select = depart.index(min(depart))
                    select_dep1.append(depart[select])
                    select_num1.append([select,s_num])
                    depart =[]
                    for j in list2:
                        depart.append(math.sqrt((pot[0] - j[0]) ** 2 + (pot[1] - j[1]) ** 2))
                    select = depart.index(min(depart))
                    select_dep2.append(depart[select])
                    select_num2.append([select,s_num])
                    depart = []
                if min(select_dep1) > min(select_dep2):
                    dex = select_dep2.index(min(select_dep2))
                    dx = list2[select_num2[dex][0]][0] - list0_s[select_num2[dex][1]][0]
                    dy = list2[select_num2[dex][0]][1] - list0_s[select_num2[dex][1]][1]
                    #print("select_dep2",select_dep2,"select_num2",select_num2,dx,dy,dex)
                else:
                    dex = select_dep1.index(min(select_dep1))
                    dx = list1[select_num1[dex][0]][0] - list0_s[select_num1[dex][1]][0]
                    dy = list1[select_num1[dex][0]][1] - list0_s[select_num1[dex][1]][1]
                    #print("select_dep1",select_dep1,"select_num1",select_num1,dx,dy,dex)
                for i in range(len(list0_s)):
                   list_pu.append([list0_s[i][0]+dx,list0_s[i][1]+dy])
                list_pu.reverse()
                select_pot =  center_line(list0_s, list_pu,data_range=20, num_of_lines=n_of_line)
        elif control == 2:
                for j in list2:
                    depart.append(math.sqrt((0 - j[0]) ** 2 + (0 - j[1]) ** 2))
                select = depart.index(min(depart))
                select_max =depart.index(max(depart))
                if len(list2)<25:
                    select = len(list2)-1
                    select_max = 0
                for j in range(select-select_max):
                    list2_s.append(list2[j+select_max])
                for i in range(dvi_num):
                    space_num = int(len(list2_s) / dvi_num)
                    s_num = i * space_num
                    pot = list2_s[(len(list2_s) - 1 - s_num)]
                    depart = []
                    for j in list0:
                        depart.append(math.sqrt((pot[0] - j[0]) ** 2 + (pot[1] - j[1]) ** 2))
                    select = depart.index(min(depart))
                    select_dep1.append(depart[select])
                    select_num1.append([select,(len(list2_s) - 1 - s_num)])
                    depart = []
                    for j in list1:
                        depart.append(math.sqrt((pot[0] - j[0]) ** 2 + (pot[1] - j[1]) ** 2))
                    select = depart.index(min(depart))
                    select_dep2.append(depart[select])
                    select_num2.append([select,(len(list2_s) - 1 - s_num)])
                    depart = []
                if min(select_dep1) > min(select_dep2):
                    dex = select_dep2.index(min(select_dep2))
                    dx = list1[select_num2[dex][0]][0] - list2_s[select_num2[dex][1]][0]
                    dy = list1[select_num2[dex][0]][1] - list2_s[select_num2[dex][1]][1]
                else:
                    dex = select_dep1.index(min(select_dep1))
                    #print("222222222222222222222222",select_num1,select_num1[dex],len(list0),len(list1),len(list2_s))
                    dx = list0[select_num1[dex][0]][0] - list2_s[select_num1[dex][1]][0]
                    dy = list0[select_num1[dex][0]][1] - list2_s[select_num1[dex][1]][1]
                for i in range(len(list2_s)):
                   list_pu.append([list2_s[i][0]+dx,list2_s[i][1]+dy])
                list_pu.reverse()
                select_pot =  center_line(list_pu,list2_s, data_range=20, num_of_lines=n_of_line)
        start2 = time.time()
        print("lei_center2_control fei 0",start2-start1)
        return select_pot

    else:
        if len(list0) > len(list2):       
            space_num = int(len(list0) / dvi_num)
            for i in range(dvi_num):
                    s_num = i * space_num
                    pot = list0[s_num]
                    for j in list1:
                        depart.append(math.sqrt((pot[0] - j[0]) ** 2 + (pot[1] - j[1]) ** 2))
                    select = depart.index(min(depart))
                    select_dep1.append(depart[select])
                    select_num1.append([select,s_num])
                    depart =[]
                    for j in list2:
                        depart.append(math.sqrt((pot[0] - j[0]) ** 2 + (pot[1] - j[1]) ** 2))
                    select = depart.index(min(depart))
                    select_dep2.append(depart[select])
                    select_num2.append([select,s_num])
                    depart = []
            if min(select_dep1) > min(select_dep2):
                    dex = select_dep2.index(min(select_dep2))
                    dx = list2[select_num2[dex][0]][0] - list0[select_num2[dex][1]][0]
                    dy = list2[select_num2[dex][0]][1] - list0[select_num2[dex][1]][1]
            else:
                    dex = select_dep1.index(min(select_dep1))
                    dx = list1[select_num1[dex][0]][0] - list0[select_num1[dex][1]][0]
                    dy = list1[select_num1[dex][0]][1] - list0[select_num1[dex][1]][1]
            for i in range(len(list0)):
                   list_pu.append([list0[i][0]+dx,list0[i][1]+dy])
            list_pu.reverse()
            select_pot =  center_line(list0, list_pu,data_range=20, num_of_lines=n_of_line)

        else:
                for i in range(dvi_num):
                    space_num = int(len(list2) / dvi_num)
                    s_num = i * space_num
                    #print ('space_num',space_num,len(list2))
                    pot = list2[(len(list2) - 1 - s_num)]
                    for j in list0:
                        depart.append(math.sqrt((pot[0] - j[0]) ** 2 + (pot[1] - j[1]) ** 2))
                    select = depart.index(min(depart))
                    select_dep1.append(depart[select])
                    select_num1.append([select,(len(list2) - 1 - s_num)])
                    depart = []
                    for j in list1:
                        depart.append(math.sqrt((pot[0] - j[0]) ** 2 + (pot[1] - j[1]) ** 2))
                    select = depart.index(min(depart))
                    select_dep2.append(depart[select])
                    select_num2.append([select,(len(list2) - 1 - s_num)])
                    depart = []
                if min(select_dep1) > min(select_dep2):
                    dex = select_dep2.index(min(select_dep2))
                    #print ('dex',dex)
                    dx = list0[select_num2[dex][0]][0] - list2[select_num2[dex][1]][0]
                    dy = list0[select_num2[dex][0]][1] - list2[select_num2[dex][1]][1]
                else:
                    dex = select_dep1.index(min(select_dep1))
                    dx = list1[select_num1[dex][0]][0] - list2[select_num1[dex][1]][0]
                    dy = list1[select_num1[dex][0]][1] - list2[select_num1[dex][1]][1]
                for i in range(len(list2)):
                   list_pu.append([list2[i][0]+dx,list2[i][1]+dy])
                list_pu.reverse()
                select_pot =  center_line(list_pu,list2, data_range=20, num_of_lines=n_of_line)
        start2 = time.time()
        print("lei_center_control=0",start2-start1)
        return select_pot
#####################################################
def turn_k_b_line(t_control,k_b_list,y_0,y_1):#根据kb，转弯方向得到直线
    start1 = time.time()
    select_path=[]
    if abs(k_b_list[0][0])<0.001:
        k_b_list[0][0] = 0.001
    if t_control == 0:
        for i in range(6):
            select_path.append([i*1.5,i*1.5*k_b_list[0][0]+k_b_list[0][1]])
    if t_control == 1:
        if k_b_list[0][0]>0:
            x_0 = (y_1-k_b_list[0][1])/k_b_list[0][0]
            for i in range(6):
                select_path.append([x_0-i*1.5,(x_0-i*1.5)*k_b_list[0][0]+k_b_list[0][1]]) 
        else:
             x_0 = (y_1-k_b_list[0][1])/k_b_list[0][0]
             for i in range(6):
                select_path.append([x_0+i*1.5,(x_0+i*1.5)*k_b_list[0][0]+k_b_list[0][1]])
    if t_control == 2:
        if k_b_list[0][0]>0:
            x_0 = (y_0-k_b_list[0][1])/k_b_list[0][0]
            for i in range(6):
                select_path.append([x_0+i*1.5,(x_0+i*1.5)*k_b_list[0][0]+k_b_list[0][1]]) 
        else:
             x_0 = (y_0-k_b_list[0][1])/k_b_list[0][0]
             for i in range(6):
                select_path.append([x_0-i*1.5,(x_0-i*1.5)*k_b_list[0][0]+k_b_list[0][1]])
    start2 =time.time()
    print("turn_k_b_line,tine",start2-start1)
    return select_path

def lei_liquntichu(list,point_gap,xiangliang_gap,a_limit,k_limit,space_line):#list为输入的类，0.5为控制离群参数控制,
    # point_gap,xiangliang_gap分别为每隔多少个点做向量，每隔多少个向量做积得到向量积,k_limit(分类依据),space_line(根据距离判断保留与否),
    list_process = []
    for i_1 in range(len(list)):
        list_o = []  #存得到的向量
        lei_xlj = [] #存计算得到的向量积
        lei_k = [] #存储根据点得到的K
        list_ce = list[i_1] #取出要处理的类
        if len(list_ce) < 1:
            break
        select_inde = [] #找到拐角
        #print("list_ce:",list_ce)
        point_num = int(len(list_ce)/point_gap) #计算可以得到多少个向量
        for i in range(point_num-1):# 计算每隔point_gap个点得到的单位点云
            sum_p = point_gap*(i+1)
            a = list_ce[sum_p][0]-list_ce[point_gap*i][0]#向量的x
            b = list_ce[sum_p][1] - list_ce[point_gap* i][1]#向两的y
            c = math.sqrt(a*a+b*b)#向量的模
            if c < 0.001:
                a = 0
                b = 0
            else:
                a = a/c
                b = b/c
            list_o.append([a, b]) #得到原始向量

        #print("list_o:",list_o)
        list_a = list_o[xiangliang_gap:-1]
        list_b = list_o[0:(len(list_o)-xiangliang_gap-1)] #将向量根据xiangliang_gap 对齐
        #print("list_a:",list_a)
        #print("list_b:",list_b)
        for i in range(len(list_b)):
            sum_xl = list_a[i][0]*list_b[i][0]+list_a[i][1]*list_b[i][1]
            lei_xlj.append(sum_xl)
            #lei_k.append(abs(get_lei_k(list_ce[point_gap*i:point_gap*(i+1)])))
        #print("lei_xlj:",lei_xlj)
        #print("lei_k:",lei_k)
        select_l = []
        while min(lei_xlj) < 0.85:
            select = lei_xlj.index(min(lei_xlj))
            select_inde.append(select)
            select_l.append( lei_xlj[select] )
            lei_xlj[select] = 1
        if (len(select_l) >0) and  (select_l[0] > a_limit):
            list_process.append(list_ce)
            #print("select_inde___j:",select_inde)
            #print("select_l__j.:",select_l)
        else:
            select_inde.sort()
            select_inde.reverse()
            select_inde.append(0)
            select_inde.reverse()
            select_inde.append(len(lei_xlj))
            list1 = []
            #print("select_inde:",select_inde)
            #print("select_l.:",select_l)
            for i in range(len(select_inde)-1):
                 lei_k.append(abs(get_lei_k(list_ce[point_gap*select_inde[i]:point_gap*select_inde[i+1]])))
            #print("lei_k ************************:",lei_k)
            for i in range(len(select_inde)-1):
                list2 = []
                #print(i,select_inde[i])
                if lei_k[i] < k_limit:  # 根据K值控制需不需要加入
                     if (select_inde[i+1]-select_inde[i]) > 2:
                        for l in range(point_gap*(select_inde[i+1]-select_inde[i])):
                            #print(point_gap*(select_inde[i])+l)
                            list2.append(list_ce[point_gap*(select_inde[i])+l])
                        #print("list2",list2)
                if len(list2)>0:
                    list1.append(list2)
            flag_r  = True
            if len(list1) > 0:
                  if list1[0][0][1] > 0:
                      flag_r = False
                      list1.reverse()
                  for i in range(len(list1)):
                      list1[i].reverse()
                  if flag_r:
                      list_p = []
                      for i in range(len(list1)):
                          if i == 0:
                              for j in range(len(list1[i])):
                                  list_p.append(list1[i][j])
                          else:
                               if (np.sqrt((list1[i][0][0] - list1[i-1][-1][0])**2+(list1[i][0][1] - list1[i-1][-1][1])**2)) < space_line :#两个线的最小距离
                                   for j in range(len(list1[i])):
                                       list_p.append(list1[i][j])
                      if len(list_p)>3:
                           list_process.append(list_p)
                  else:
                       list_p = []
                       for i in range(len(list1)):
                           if i == 0:
                                for j in range(len(list1[i])):
                                    list_p.append(list1[i][j])
                           else:
                               if  (np.sqrt((list1[i][0][0] - list1[i-1][-1][0])**2+(list1[i][0][1] - list1[i-1][-1][1])**2)) < space_line :#两个线的最小距离
                                     for j in range(len(list1[i])):
                                         list_p.append(list1[i][j])
                                     list_p.reverse()
                       if len(list_p)>3:
                           list_process.append(list_p)
    #print("list_process:",list_process)
    return list_process
def tunnel_center_line(pcd_all, data_range=20, num_of_lines=5, draw=False, control=0, turn_control=0,l_long=4.0,dd_l=2,dd_k=2.5,k_d_line=1.5): #1右，2左，0保持control = True,turn_control控制转弯方向
    #print("**************************************************************************************************************",list,"*************************************************************************************")
    '''
    if draw2:
       list = lei_liquntichu(list, point_gap, xiangliang_gap,a_limit, k_limit, space_line)
       lib_JuLei.plot_list(list)
    else:
    '''
    #############################################
    start1 =time.time()
    list = pcd_all
    #lib_JuLei.plot_list(list)
    num = len(list)
    #print("&&&&&&num&&&&&",num,"YYYYYYYYYYYYYYYYYYYYYYYYYYYY")
    c_control = control
    ###############################################
    start2= time.time()
    if num == 1:
        depart = []
        for j in list[0]:
            depart.append(math.sqrt((0 - j[0]) ** 2 + (0 - j[1]) ** 2))
        select = depart.index(min(depart))
        if ( list[0][select][1]<0):
            c_control = 1
        else:
            c_control = 2
        #######################################
        lei_1_t = time.time()
        print("1,lei_num1_____________",start2-start1,lei_1_t-start2)
        return lei_center(dd_l,dd_k,list[0], list[0], list[0],c_control,10)
    elif num == 2:
        lei_2_t1 = time.time()
        k_class = []
        b_class = []
        r_class = []
        for j in range(num):  # 聚类数目不是二（主要讨论聚类大于二的情况）
            a, b, r = K_leastsquare(list[j])  # 得到每一聚类的K值
            k_class.append(a)
            b_class.append(b)
            r_class.append(r)
        sida_01 = np.arctan(abs((k_class[0] - k_class[1]) / (1 + k_class[0] * k_class[1])))
        lei_2_t2 = time.time()
        if (turn_control == 0) and (sida_01 < 30) and (r_class[0] > 0.65 and r_class[1] >0.65):
            if len(list[0]) > 6 and len(list[1]) > 6:
                lei_2_t3 = time.time()
                print("2,len(list[0]) > 6 and len(list[1]) > 6:",start2-start1,lei_2_t1-start2,lei_2_t2-lei_2_t1,lei_2_t3-lei_2_t2)
                return center_line(list[0], list[1], data_range, num_of_lines-1)
        else:
            lei_2_t3 = time.time()
            dvi_num = num_of_lines+6
            depart = []
            select_pot = []
            select_dep = []
            select_num=[]
            list_pu = []
            lei_2_t4 = time.time()
            #if r_class[1] < r_class[0]:
            if control == 1:
                space_num = int(len(list[0]) / dvi_num)
                for i in range(dvi_num):
                    s_num = i * space_num
                    pot = list[0][s_num]
                    for j in list[1]:
                        depart.append(math.sqrt((pot[0] - j[0]) ** 2 + (pot[1] - j[1]) ** 2))
                    select = depart.index(min(depart))
                    select_dep.append(depart[select])
                    select_num.append([select,s_num])
                    #print("result6:", select, list[1][select])
                    depart = []
                #print("select_dep",select_dep,"select_num",select_num)
                dex = select_dep.index(min(select_dep))
                #print ('dex',dex)
                dx = list[1][select_num[dex][0]][0] - list[0][select_num[dex][1]][0]
                dy = list[1][select_num[dex][0]][1] - list[0][select_num[dex][1]][1]
                for i in range(len(list[0])):
                   list_pu.append([list[0][i][0]+dx,list[0][i][1]+dy])
                list_pu.reverse()
                select_pot =  center_line(list[0],list_pu, data_range, num_of_lines)
                print("2,control=1,****************************************????????????????????????????????????????????????????????????")
                """
                d_min = min(select_dep)
                for i in range(len(select_num)):
                    if select_dep[i] < 1+d_min:
                        select_pot.append([(list[0][ select_num[i][1]][0] + list[1][select_num[i][0]][0]) / 2, (list[0][ select_num[i][1]][1] + list[1][select_num[i][0]][1]) / 2])
                print("d_min ",select_pot)
                """
            elif control == 2:
                for i in range(dvi_num):
                    space_num = int(len(list[1]) / dvi_num)
                    s_num = i * space_num
                    pot = list[1][(len(list[1]) - 1 - s_num)]
                    for j in list[0]:
                        depart.append(math.sqrt((pot[0] - j[0]) ** 2 + (pot[1] - j[1]) ** 2))
                    select = depart.index(min(depart))
                    select_dep.append(depart[select])
                    select_num.append([select,(len(list[1]) - 1 - s_num)])
                    #print("result7:", select, list[0][select])
                    depart = []
                #print("select_dep",select_dep,"select_num",select_num)
                dex = select_dep.index(min(select_dep))
                #print ('dex',dex)
                dx = list[0][select_num[dex][0]][0] - list[1][select_num[dex][1]][0]
                dy = list[0][select_num[dex][0]][1] - list[1][select_num[dex][1]][1]
                for i in range(len(list[1])):
                   list_pu.append([list[1][i][0]+dx,list[1][i][1]+dy])
                list_pu.reverse()
                select_pot =  center_line(list_pu,list[1], data_range, num_of_lines)
                print("2,control=2,****************************************??????????????????????????????????????????????????????")
                """
                d_min = min(select_dep)
                #dx = list0[select_num2[dex][0]][0] - list2[select_num2[dex][1]][0]
               #dy = list0[select_num2[dex][0]][1] - list2[select_num2[dex][1]][1]
                for i in range(len(select_num)):
                    if select_dep[i] < 1+d_min:
                        select_pot.append([(list[1][ select_num[i][1]][0] + list[0][select_num[i][0]][0]) / 2, (list[1][ select_num[i][1]][1] + list[0][select_num[i][0]][1]) / 2])
                print("d_min ",select_pot)
                """
            else:
               k_b =  center_line(list[0], list[1], data_range, 1, draw)#如果直行直接svm（不分段）
               select_pot=turn_k_b_line(turn_control,k_b,k_d_line,-k_d_line)
               print("如果直行直接svm（不fen)********************************************************************")
            lei_2_t5 = time.time()
            print("2,tun_ceter_line,yi bian wei chun :",start2-start1,lei_2_t1-start2,lei_2_t2-lei_2_t1,lei_2_t3-lei_2_t2,lei_2_t4-lei_2_t3,lei_2_t5-lei_2_t4)
            return select_pot
    else:
        lei_n_t1 = time.time()
        lines = []
        k_class = []
        b_class = []
        r_class = []
        for j in range(num):   # 聚类数目不是二（主要讨论聚类大于二的情况）
            a, b, r = K_leastsquare(list[j])  # 得到每一聚类的K值
            k_class.append(a)
            b_class.append(b) 
            r_class.append(r)
        if num == 3:  #  分情况讨论
            lei_3_t1 = time.time()           
            sida_01 = np.arctan(abs((k_class[0] - k_class[1]) / (1 + k_class[0] * k_class[1])))
            sida_02 = np.arctan(abs((k_class[0] - k_class[2]) / (1 + k_class[0] * k_class[2])))
            sida_12 = np.arctan(abs((k_class[1] - k_class[2]) / (1 + k_class[1] * k_class[2]))) #求得两两直线的夹角
            # if abs(K_class[0]) > abs(K_class[2]):
                # k_max = K_class[0]  # 取绝对值大的K值，得到大的巷道宽
            # else:
                # k_max = K_class[2]
            # dao_wide = (b_class[0] - b_class[2]) / (1 + pow(1/k_max, 2))
            #print('r--', r_class, "sida:", sida_01, sida_02, sida_12)
            lei_3_t2 = time.time() 
            if ((sida_01/3.1415926)*180 < 15 and (sida_12/3.1415926)*180 < 15) and (
                     r_class[2]  > 0.8 and r_class[0] > 0.8 and r_class[2] > 0.8) and (control == 0): #无岔道口，根据夹角的值和截距判断是否同一类
                #print("noturn and herge:")
                if abs(b_class[0]-b_class[1]) <= abs(b_class[1]-b_class[2]):
                    for i in list[1]:
                        list[0].append(i)
                    list1 = list[0]
                    list2 = list[2]
                else:
                    for i in list[2]:
                        list[1].append(i)
                    list1 = list[0]
                    list2 = list[1]
                lei_3_t3= time.time()
                print("3,herge",start2-start1,lei_n_t1-start2,lei_3_t1-lei_n_t1,lei_3_t2-lei_3_t2,lei_3_t3-lei_3_t2)
                return center_line(list1, list2,data_range, num_of_lines)

                '''
            elif r_class[0] > 0.9 and r_class[1] > 0.9 and (
                    ((sida_01/3.1415926)*180 < 45 or (sida_12/3.1415926)*180 < 45) and r_class[2] < 0.9):#岔道口#最小转角大于等于90,得到两条直线夹角（钝角）接近Y形
                point_02 = center_line(list[0], list[2],20,5) #先用左右两类求到一条中心线，根据中心线长度判断
                if len(point_02) > 2:
                    if np.sqrt((point_02[0][0] - point_02[-1][0]) ** 2 + (
                            (point_02[0][1] - point_02[-1][1]) ** 2)) < l_long:
                        if dao_wide > 6:  # 根据常规移2.5m（xiangdao_wide>5)否侧移dao_wide/2
                            wei_yi = 3
                        else:
                            wei_yi = dao_wide / 2
                        if wei_yi < 1.8:
                            wei_yi = 1.8
                        if control:
                            point = lei_dian(list[0], 3)
                            dx = wei_yi / (1 + pow(1 / K_class[0], 2))
                            dy = -wei_yi / (1 + pow(K_class[0], 2))
                        else:
                            point = lei_dian(list[2], 3)
                            dx = wei_yi / (1 + pow(1 / K_class[2], 2))
                            dy = wei_yi / (1 + pow(K_class[2], 2))
                        for j in point:
                            j[0] += j[0] + dx
                            j[1] += j[1] + dy
                            return point
                    else:
                        return point_02
                else:
                    if dao_wide > 6:  # 根据常规移2.5m（xiangdao_wide>5)否侧移dao_wide/2
                        wei_yi = 3
                    else:
                        wei_yi = dao_wide / 2
                    if wei_yi < 1.8:
                        wei_yi = 1.8
                    if control:
                        point = lei_dian(list[0], 3)
                        dx = wei_yi / (1 + pow(1 / K_class[0], 2))
                        dy = -wei_yi / (1 + pow(K_class[0], 2))
                    else:
                        point = lei_dian(list[2], 3)
                        dx = wei_yi / (1 + pow(1 / K_class[2], 2))
                        dy = wei_yi / (1 + pow(K_class[2], 2))
                    for j in point:
                        j[0] += j[0] + dx
                        j[1] += j[1] + dy
                        return point
                 '''
            else:
                if turn_control == 0:
                    point_02 = center_line(list[0], list[2],data_range, num_of_lines-2)  # 先用左右两类求到一条中心线，根据中心线长度判断
                    if len(point_02) > 2:
                        if np.sqrt((point_02[0][0] - point_02[-1][0]) ** 2 + ((point_02[0][1] - point_02[-1][1]) ** 2)) < l_long:
                            #return lei_center2(list[0], list[1],list[2],control,10,num_of_lines
                            lei_3_t3= time.time()
                            print("3,zuoyou",start2-start1,lei_n_t1-start2,lei_3_t1-lei_n_t1,lei_3_t2-lei_3_t2,lei_3_t3-lei_3_t2)
                            return turn_k_b_line(turn_control,center_line(list[0], list[2], data_range, 1),k_d_line,-k_d_line)
                        else:
                            lei_3_t3= time.time()
                            print("3,lianglei",start2-start1,lei_n_t1-start2,lei_3_t1-lei_n_t1,lei_3_t2-lei_3_t2,lei_3_t3-lei_3_t2)                            
                            return point_02
                    else:
                        lei_3_t3= time.time()
                        print("3,zuoyou",start2-start1,lei_n_t1-start2,lei_3_t1-lei_n_t1,lei_3_t2-lei_3_t2,lei_3_t3-lei_3_t2) 
                        #return lei_center2(list[0], list[0],list[2],control,10,num_of_lines)
                        return turn_k_b_line(turn_control,center_line(list[0], list[2], data_range, 1),k_d_line,-k_d_line)
                elif turn_control == 1:
                    lei_3_t3= time.time()
                    print("3,zuoyou",start2-start1,lei_n_t1-start2,lei_3_t1-lei_n_t1,lei_3_t2-lei_3_t2,lei_3_t3-lei_3_t2) 
                    return turn_k_b_line(turn_control,center_line(list[0], list[1], data_range, 1),k_d_line,-k_d_line)
                elif turn_control == 2:
                    lei_3_t3= time.time()
                    print("3,zuoyou",start2-start1,lei_n_t1-start2,lei_3_t1-lei_n_t1,lei_3_t2-lei_3_t2,lei_3_t3-lei_3_t2)
                    return turn_k_b_line(turn_control,center_line(list[2], list[1], data_range, 1),k_d_line,-k_d_line)
                """
                   if dao_wide > 6:  # 根据常规移2.5m（xiangdao_wide>5)否侧移dao_wide/2
                        wei_yi = dao_wide / 2
                    else:
                        wei_yi = dao_wide / 2
                    if wei_yi < 1.8:
                        wei_yi = 1.8
                    k_center, b_center, r_center = K_leastsquare(point_02)
                    x_cross, y_cross = get_crossing_point(k_center, b_center, K_class[1], b_class[1])
                    if K_class[1] > 0:
                        dy1 =-wei_yi/(1+pow(K_class[1], 2))
                    else:
                        dy1 = -wei_yi/(1 + pow(K_class[1], 2))
                    dx1 = -wei_yi/(1 + pow(1/K_class[1], 2))
                    if control:  #右转
                        if K_class[1] > 0:
                            dx2 = -wei_yi / (1 + pow(K_class[1], 2))
                        else:
                            dx2 = wei_yi / (1 + pow(K_class[1], 2))
                        dy2 = -wei_yi / (1 + pow(1/K_class[1], 2))
                        x_yuan = x_cross+2*dx1+dx2
                        y_yuan = y_cross+2*dy1+dy2
                        x_jie = x_cross+dx1+dx2
                        y_jie = y_cross+dy1+dy2
                    else:  #左转
                        if K_class[1] > 0:
                            dx2 = wei_yi / (1 + pow(K_class[1], 2))
                        else:
                            dx2 =- wei_yi / (1 + pow(K_class[1], 2))
                        dy2 = wei_yi / (1 + pow(K_class[2], 2))
                        x_yuan = x_cross + 2 * dx1 + dx2
                        y_yuan = y_cross + 2 * dy1 + dy2#找到聚类二拟合直线和0——2直线交点对应的圆心
                        x_jie = x_cross + dx1 + dx2#找到类二拟合直线沿垂直方向移动wei_yi后圆心对应的切点
                        y_jie = y_cross + dy1 + dy2
                    point_cri = [[(x_jie+0.4*dx2), (y_jie+0.4*dy2)]]
                    angle_increment = (22.5/180)*3.1415926
                    for i in range(5):
                        x_cri = x_jie+(1-np.cos(angle_increment*(i+1)))*dx1-dx2*np.sin(angle_increment*(i+1))
                        y_cri = y_jie+(1-np.cos(angle_increment*(i+1)))*dy1-dy2*np.sin(angle_increment*(i+1))
                        point_cri.append([x_cri, y_cri])
                    point_cri.append(point_02[-1])
                    point_cri.append(point_02[0])
                    return point_cri
                """

        else:
            lei_n_t2= time.time()
            print("n",start2-start1,lei_n_t1-start2,lei_n_t2-lei_n_t1)
            if turn_control == 0:
                return  turn_k_b_line(turn_control,center_line(list[0], list[num-1], data_range, 1),k_d_line,-k_d_line)
            elif turn_control == 1:
                return turn_k_b_line(turn_control,center_line(list[0], list[1], data_range, 1),k_d_line,-k_d_line)
            elif turn_control == 2:
                return turn_k_b_line(turn_control,center_line(list[num-2], list[num-1], data_range, 1),k_d_line,-k_d_line)
            """path_others = []
            for i in range(num):
                if i == num - 1:   #取最后一个聚类
                    X_0 = np.array(list[i]) #data
                    y_0 = np.zeros([len(X_0)], dtype='int') #lable
                    X_1 = np.array(list[0])
                    y_1 = np.ones([len(X_1)], dtype='int')
                    X = np.r_[X_0, X_1] #data
                    y = np.r_[y_0, y_1] #lable
                    k, b = run_svm(X, y, data_range, False)
                    lines.append([k, b]) #get k,b
                    continue
                X_0 = np.array(list[i])#两个相邻的聚类做svm，得到中心线
                y_0 = np.zeros([len(X_0)], dtype='int')
                X_1 = np.array(list[i + 1])
                y_1 = np.ones([len(X_1)], dtype='int')
                X = np.r_[X_0, X_1]
                y = np.r_[y_0, y_1]
                k, b = run_svm(X, y, data_range, False)
                lines.append([k, b])

        # 选择哪一条
            k = lines[num-1][0]
            b = lines[num-1][1]#选左右两个类得到一个K,B
        #！  ！！！！！这里是如何选择的，如何保证选择的k,b是由所在路径左右两边的点云产生的，而且此处的点都是由一个k,b求得的，对于弯曲路经是否无用？
        #可以由接口控制他选择的路径。
            y_arr = np.array((-3, -6, -9))
            x_arr = (y_arr - b) / k
            path_point_arr = np.c_[x_arr, y_arr] ### 拼接出所求的中心点得到[ [x,y] [x,y]]的形式
        # ##np.c_是按行连接两个矩阵，就是把两矩阵左右相加，要求行数相等
            #print(path_point_arr.shape)
            path_others = path_point_arr.tolist()
            #print(lines)
            #print(path_point)
            return path_others
    """

if __name__ == '__main__':
    pass
