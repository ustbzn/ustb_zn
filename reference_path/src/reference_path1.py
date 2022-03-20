#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''reference_path ROS Node'''
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from reference_path.msg import uwb
from reference_path.msg import Frame
import lib_JuLei
import lib_ZhongXinXianQu
import math
import time

#*********************************************************************************# 触须算法的相关库与参数
'''local_path_planner ROS Node'''
#import rospy
#import numpy as np
#import math
#from std_msgs.msg import String
#from nav_msgs.msg import Path
#from geometry_msgs.msg import PoseStamped
#from webots_ros.msg import Float64Stamped
from std_msgs.msg import Float32
import lib_local_path
import lib_cost_cal

gamma = 0.0
vehicle_speed = 10/3.6
min_line_gap = 1.2 #控制得到的触须离类的最短距离
uwb_count = [0,0]#对出现的情况计数
count_set = 3 #情况计数大于3才对转弯，摸墙参数设定


"""
def local_path_3_offline(gamma, path_reference, v_online):

    gamma_range=[-0.3, 0.3]
    
    gamma_arr = np.linspace(gamma_range[0], gamma_range[-1], 30)
    min_id = np.argmin(np.abs(gamma_arr - gamma))
    # arr = np.array(list_arr)
    # global arr
    arr = np.load('paths.npy')
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
"""
def cal_length(path_reference):
    len_p = math.sqrt(path_reference[-1, 0] ** 2 + path_reference[-1, 1] ** 2)
    return len_p

#*********************************************************************************#触须算法部分结束
# 参数设置
DATA_RANGE = 5#DATA_RANGE = 20  # 设定截取参与运算的激光点云范围
NUM_OF_SUBSECTION = 4  # 设定分段数
ELIMINATE_TRESHOLD = 2.5  # 设定剔除离群点的阈值(m) 1
SELFELIMINATE_TRESHOLD =1.2 #SELFELIMINATE_TRESHOLD =4# 设定濾波半徑(m)
POINT_CLOUD_DECIMALS = 3  # 设定点云数据小数点后保留几位
julei_number_control = 10#控制聚类的最小点数
angle_increment = 0.02 #二位点云的递增角
julei_d = 3.0 #聚类的距离S 1.5
plot_con = True #画图控制参数
plot_con2 =False   #画图控制参数(dui lei chu li hou )
control_turn_uwb= 0  #控制转弯方向:1-right,2-left,0-direct
zhixiang_set = 0 #对于直道上的大拐弯。
line_l =2 #  中心线长度控制1.5
front_back_para = 0 #正行设为一，倒行设为一。
control_u= 2#控制摸那边的墙:1：摸右边，2：摸左边
#start = time.time()  #开始计时 
def callback_laser(laser):
    start1 = time.time()
    '''reference_path Callback Function'''
    # laserscan数据解算
    pcd = []
    pcd1 = []

    for i in range(len(laser.ranges)):
        x = laser.ranges[i] * math.sin(-0.5235987668+i * angle_increment)
        y = -laser.ranges[i] * math.cos(-0.5235987668+i * angle_increment)
        pcd.append([x, y])
    print("laser.ranges:", len(laser.ranges))
    # 点云数据格式化
    pcd = np.array(pcd)
    pcd = np.around(pcd, decimals=POINT_CLOUD_DECIMALS)
    pcd = pcd.astype(np.float32)
    pcd1 = pcd[30:211]
    start2 = time.time()
    #print("pcd_1:",pcd)
    # 点云数据处理
    '''
    ELIMINATE_TRESHOLD  # 设定剔除离群点的阈值
    SELFELIMINATE_TRESHOLD # 设定濾波半徑
    '''
    pcd = lib_JuLei.ti_chu_li_qun_dian(pcd1, treshold=ELIMINATE_TRESHOLD,treshold_de=SELFELIMINATE_TRESHOLD)
    start3 = time.time()
    #print("pcd_2:",pcd)
    '''
    julei_number_control = 6 #控制聚类的最小点数
    plot_con = True #画图控制参数
    julei_d = 0.7 #聚类的距离S
    '''
    pcd = lib_JuLei. ju_lei(pcd, threshold=julei_d,julei_min_number=julei_number_control)
    start4= time.time()
    #print("pcd_3",pcd)
    '''
    DATA_RANGE = 6#DATA_RANGE = 20  # 设定截取参与运算的激光点云范围
   NUM_OF_SUBSECTION = 4  # 设定分段数
    '''
    #T = end-start
    global control_u,control_turn_uwb,zhixiang_set
    reference_point = lib_ZhongXinXianQu.tunnel_center_line(pcd, data_range=DATA_RANGE, num_of_lines=NUM_OF_SUBSECTION, draw=plot_con, control=control_u,turn_control=control_turn_uwb, l_long=line_l, dd_l=2,dd_k=2,k_d_line= 1.5)    ##1右，2左，0保持control = True,turn_control控制转弯方向
    #print("reference_point", reference_point)         reference_point =  [[x0,y0],[x1,y1]]
    reference_point1 = []
    if len(reference_point) > 2:
        reference_point1 = reference_point
    elif len(reference_point) == 2:
        reference_point1.append([0,0])
        for i in reference_point:
            reference_point1.append(i)
    else:
        reference_point1.append([1.2,0])
        reference_point1.append(reference_point[0])
        reference_point1.append([2*reference_point[0][0],2*reference_point[0][1]])
    x = reference_point1[0][0]
    y = reference_point1[0][1]
    if zhixiang_set == 3:
            for i in reference_point1:
                i[0]=i[0]-x
                i[1]=i[1]-y
                print("*******************************************************____________")
    start5= time.time()
    #*********************************************************************************#触须算法处理
    path_reference = np.array(reference_point1)
    len_p = cal_length(path_reference)
    v_online = len_p / 3.0
    global gamma,vehicle_speed
    if vehicle_speed>2.5:
        vehicle_speed=10/3.6
    elif vehicle_speed > 2:
        vehicle_speed=8/3.6
    else:
         vehicle_speed=6/3.6
    # X, Y = lib_local_path.local_path(gamma, path_reference, v_online)
    # X, Y, d_gamma = lib_local_path.local_path_3(gamma, path_reference, v_online)
    X, Y, d_gamma, cost_steady, cost_deviation = lib_local_path.local_path_3(gamma, path_reference,vehicle_speed,pcd,min_line_gap)
    # X, Y, d_gamma = local_path_3_offline(gamma, path_reference, 10/3.6)
    #################reference_point2 = get_range_poinnt(0.5,15,reference_point1,0.8,10)
    #*********************************************************************************#选到路径
    #编写message
    start6 = time.time()
    path = Path()
    path.header.frame_id = '/laser_link'
    ##
    path.header.stamp = rospy.Time.now()
    for i in range(len(reference_point1)):
        poseStamped = PoseStamped()
        poseStamped.pose.position.x = reference_point1[i][0]
        poseStamped.pose.position.y = reference_point1[i][1]
        path.poses.append(poseStamped)

    reference_path_pub = rospy.Publisher('reference_path', Path, queue_size=1)
    reference_path_pub.publish(path)
    #编写message2
    path1 = Path()
    path1.header.frame_id = '/laser_link'
    ##
    path1.header.stamp = rospy.Time.now()
    #gap_num = max(int(X.shape[0]/10),1)
    #num_path = min(X.shape[0],10)
    gap_num = 1
    gap_point = chazhi_poinnt(X,Y,gap_num)
    num_path = len(gap_point)
    print ("gap_pointgap_pointgap_pointgap_pointgap_pointgap_pointgap_pointgap_pointgap_pointgap_pointgap_pointgap_point",X.shape[0],len(gap_point))
    for i in range(num_path):
        poseStamped = PoseStamped()
        poseStamped.pose.position.x = gap_point[i*gap_num][0]
        poseStamped.pose.position.y = gap_point[i*gap_num][1]
        if i < num_path-1:
            poseStamped.pose.position.z = math.atan((gap_point[(i+1)*gap_num][1] - gap_point[i*gap_num][1]) / ( gap_point[(i+1)*gap_num][0]-  gap_point[i*gap_num][0]))
        else:
            poseStamped.pose.position.z = math.atan((gap_point[i*gap_num][1]- gap_point[(i-1)*gap_num][1]) / ( gap_point[i*gap_num][0] -  gap_point[(i-1)*gap_num][0]))
        path1.poses.append(poseStamped)

    reference_path_pub1 = rospy.Publisher('ladar_vehicle_coordinate', Path, queue_size=1)
    #print("ladar_vehicle_coordinate",path1)
    reference_path_pub1.publish(path1)
    end= time.time()
    print("time_huizong######################################",start2-start1,start3-start2,start4-start3,start5-start4,start6-start5,end-start6,end-start1)
    if (end-start1) > 0.1:
        print("##################################################################~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&")


# 使用元组存储所有基站的ID,列表存储对应距离
Teminal =(1103,996,975,1093,1063)
def select_ID(T_ID):
    if T_ID ==Teminal[0]:
        return 0
    elif T_ID ==Teminal[1]:
        return 1
    elif T_ID ==Teminal[2]:
        return 2
    elif T_ID ==Teminal[3]:
        return 3
    elif T_ID ==Teminal[4]:
        return 4
    else:
        return 5
def callback_uwb(uwb_p):
    Teminal_T=[0,0,0,0,0,0] #最后一位用于处理没有收到数据时的接受
    Teminal_T[select_ID(uwb_p. terminal_1)] = uwb_p.T1
    Teminal_T[select_ID(uwb_p. terminal_2)] = uwb_p.T2
    Teminal_T[select_ID(uwb_p. terminal_3)] = uwb_p.T3
    T_1103=Teminal_T[0] 
    T_996=Teminal_T[1]
    T_975=Teminal_T[2]
    T_1093=Teminal_T[3]
    T_1063=Teminal_T[4] 
    T_count = 0
    global control_u,control_turn_uwb,uwb_count,count_set ,zhixiang_set,front_back_para
    if front_back_para == 0:
        if (T_975 > 10)or(T_1093  > 10)or(T_1063  > 10):#Teminal =(1103,996,975,1093,1063)
            if ((T_975>1500 )and(T_975<3000 ))and((T_1093>0)and(T_1093<2000)):
                if ((control_u != 1)or(control_turn_uwb!=0))and(uwb_count[0]==0):
                    uwb_count[1]= uwb_count[1]+1
                else:
                    uwb_count[1] = 0
                uwb_count[0] = 0
                if uwb_count[1] > count_set: 
                    control_u = 1
                    control_turn_uwb=0
            elif ((T_975>0 )and(T_975<2000 ))and((T_1093>2000)and(T_1093<3500)):
                if ((control_u != 1)or(control_turn_uwb!=1))and(uwb_count[0]==1):
                    uwb_count[1]= uwb_count[1]+1
                else:
                    uwb_count[1] = 0
                uwb_count[0] = 1
                if uwb_count[1] > count_set: 
                    control_u=1
                    control_turn_uwb=1
            elif ((T_975>4500 )and(T_975<4000 ))and((T_1093>0)and(T_1093<1000)):
                if ((control_u != 1)or(control_turn_uwb!=1))and(uwb_count[0]==2):
                    uwb_count[1]= uwb_count[1]+1
                else:
                    uwb_count[1] = 0
                uwb_count[0] = 2
                if uwb_count[1] > count_set:
                    control_turn_uwb=1
                    control_u = 1
            elif ((T_1063>3500 )and(T_1063<4500 ))and((T_1093>0)and(T_1093<1000)):
                if (control_turn_uwb!=2)and(uwb_count[0]==3):
                    uwb_count[1]= uwb_count[1]+1
                else:
                    uwb_count[1] = 0
                uwb_count[0] = 3
                if uwb_count[1] > count_set:
                    control_turn_uwb=2
            elif ((T_1063>0 )and(T_1063<3500 ))and((T_1093>1000)and(T_1093<2000)):
                if ((control_u != 2)or(control_turn_uwb!=0))and(uwb_count[0]==4):
                    uwb_count[1]= uwb_count[1]+1
                else:
                    uwb_count[1] = 0
                uwb_count[0] = 4
                if uwb_count[1] > count_set:
                    control_u=2
                    control_turn_uwb=0
            elif ((T_1063>0 )and(T_1063<1500 )):
                if ((control_u != 2)or(control_turn_uwb!=0))and(uwb_count[0]==5):
                    uwb_count[1]= uwb_count[1]+1
                else:
                    uwb_count[1] = 0
                uwb_count[0] = 5
                if uwb_count[1] > count_set:
                    control_u=2
                    control_turn_uwb=0
            elif ((T_1063>1500 )and(T_1063<4500 )):
                if ((control_u != 2)or(control_turn_uwb!=2))and(uwb_count[0]==6):
                    uwb_count[1]= uwb_count[1]+1
                else:
                    uwb_count[1] = 0
                uwb_count[0] = 6
                if uwb_count[1] > count_set: 
                    control_u=1
                    control_turn_uwb=1
            elif (T_975>400)and(T_996<11800):
                if ((control_u != 2)or(control_turn_uwb!=0))and(uwb_count[0]==7):
                    uwb_count[1]= uwb_count[1]+1
                else:
                    uwb_count[1] = 0
                uwb_count[0] = 7
                if uwb_count[1] > count_set:
                    control_u=2
                    control_turn_uwb=0
            elif (T_975>1500)and(T_975<6000):
                if ((control_u != 1)or(control_turn_uwb!=0))and(uwb_count[0]==8):
                    uwb_count[1]= uwb_count[1]+1
                else:
                    uwb_count[1] = 0
                uwb_count[0] = 8
                if uwb_count[1] > count_set:
                    control_u = 1
                    control_turn_uwb=0
        elif(T_996>10):
            if ((T_996>0 )and(T_996<1500 )):
                if ((control_u != 2)or(control_turn_uwb!=2))and(uwb_count[0]==9):
                    uwb_count[1]= uwb_count[1]+1
                else:
                    uwb_count[1] = 0
                uwb_count[0] = 9
                if uwb_count[1] > count_set:
                    control_u=2
                    control_turn_uwb=2
            elif (T_996>1500 ):
                if (control_turn_uwb!=0)and(uwb_count[0]==10):
                    uwb_count[1]= uwb_count[1]+1
                else:
                    uwb_count[1] = 0
                uwb_count[0] = 10
                if uwb_count[1] > count_set:
                    control_turn_uwb=0
                    control_u=2
        if ((T_996>2000 )and(T_996<3300)):
            zhixiang_set = 1
        elif ((T_996>200 )and(T_996<2000)):
            zhixiang_set = 0
    ########################################################hou che 
    elif front_back_para == 1:
        if (T_1103 > 10):#Teminal =(1103,996,975,1093,1063)
            if (T_1103>10 )and(T_1103<3700 ):
                if ((control_u != 1)or(control_turn_uwb!=0))and(uwb_count[0]==0):
                    uwb_count[1]= uwb_count[1]+1
                else:
                    uwb_count[1] = 0
                uwb_count[0] = 0
                if uwb_count[1] > count_set: 
                    control_u = 1
                    control_turn_uwb=0
        elif (T_996 > 50)or(T_975>10):
            if (T_996 > 0 )and(T_996<1500 ):
                if ((control_u != 1)or(control_turn_uwb!=1))and(uwb_count[0]==1):
                    uwb_count[1]= uwb_count[1]+1
                else:
                    uwb_count[1] = 0
                uwb_count[0] = 1
                if uwb_count[1] > count_set: 
                    control_u = 1
                    control_turn_uwb=1
            if ((T_996 > 1500 )and(T_975>500 ))or((T_996 > 1500 )and(T_996>2500 )): 
                if ((control_u != 1)or(control_turn_uwb!=0))and(uwb_count[0]==2):
                    uwb_count[1]= uwb_count[1]+1
                else:
                    uwb_count[1] = 0
                uwb_count[0] = 2
                if uwb_count[1] > count_set: 
                    control_u = 1
                    control_turn_uwb=0
        elif (T_975 > 10)or(T_1093  > 10)or(T_1063  > 10):
            if (T_975>0 )and(T_975<1500 ):
                if ((control_u != 2)or(control_turn_uwb!=2))and(uwb_count[0]==3):
                    uwb_count[1]= uwb_count[1]+1
                else:
                    uwb_count[1] = 0
                uwb_count[0] = 3
                if uwb_count[1] > count_set: 
                    control_u=2
                    control_turn_uwb=2
            if (T_975>1500 )and((T_1093>1700)and(T_1093<3000)):
                if ((control_u != 2)or(control_turn_uwb!=2))and(uwb_count[0]==4):
                    uwb_count[1]= uwb_count[1]+1
                else:
                    uwb_count[1] = 0
                uwb_count[0] = 4
                if uwb_count[1] > count_set: 
                    control_u=2
                    control_turn_uwb=2
            if (T_1093<1700)and(T_1093>10):
                if ((control_u != 1)or(control_turn_uwb!=1))and(uwb_count[0]==5):
                    uwb_count[1]= uwb_count[1]+1
                else:
                    uwb_count[1] = 0
                uwb_count[0] = 5
                if uwb_count[1] > count_set: 
                    control_u=1
                    control_turn_uwb=1
            if (T_1093>1700)and(T_1063>1400):
                if ((control_u != 1)or(control_turn_uwb!=0))and(uwb_count[0]==6):
                    uwb_count[1]= uwb_count[1]+1
                else:
                    uwb_count[1] = 0
                uwb_count[0] = 6
                if uwb_count[1] > count_set: 
                    control_u=1
                    control_turn_uwb=0
            if (T_1063>0)and(T_1063<1500):
                if ((control_u != 2)or(control_turn_uwb!=2))and(uwb_count[0]==6):
                    uwb_count[1]= uwb_count[1]+1
                else:
                    uwb_count[1] = 0
                uwb_count[0] = 6
                if uwb_count[1] > count_set: 
                    control_u=2
                    control_turn_uwb=2
        if ((T_996>2000 )and(T_996<3300)):
            zhixiang_set = 1
        elif ((T_996>200 )and(T_996<2000)):
            zhixiang_set = 0

        #turn_control = 2 # 外部接口用于控制车辆转向方向
        #turn_control = 1 
   # if T > 100000:
    #turn_control = 1
    print("********************************control***********************turn_control", control_u,control_turn_uwb)
def callback_gamma_v(data):# 根据转角传感器实时更新车辆的转角
    global gamma,vehicle_speed
    id=data.id
    if (id == 218050563):
        if (ord(data.data[5])< 100):
                car_speed = 0.1 * ord(data.data[5])
        elif (100 <ord(data.data[5]) and ord(data.data[5] < 200)):
                car_speed = 100 * 0.1 + (ord(data.data[5]) - 100) * 0.2
        else:
                car_speed = 100 * 0.1 + 100 * 0.2 + (ord(data.data[5]) - 200) * 1
        car_speed = car_speed / 3.6
        vehicle_speed = car_speed
        #print( "speed:" ,car_speed ,"m/s" )
    if (id == 497):
        jiaojie_angle = (42.65-((ord(data.data[1])*256+ord(data.data[0]))*0.022-86))*math.pi/180
        gamma =jiaojie_angle
        #print("********************************turn_control_uwb***********************", jiaojie_angle*180,ord(data.data[0]),ord(data.data[1]),data.data[0],data.data[1])

def chazhi_poinnt(X1,Y1,point_number): #在两点之间插点
    refe_point = []
    x_point = 0 
    y_point = 0
    x_gap = 0
    y_gap = 0
    for i in range(X1.shape[0]-1):
        x_point =X1[i+1]-X1[i]
        y_point=Y1[i+1]-Y1[i]
        x_gap = x_point /(point_number+1)
        y_gap = y_point /(point_number+1)      
        for j in range(point_number+1):
            refe_point.append([(X1[i]+x_gap*j),(Y1[i]+y_gap*j)])
    return refe_point

def listener():
    '''reference_path Subscriber'''
    rospy.init_node('reference_path')
    rospy.Subscriber("/scan_laser_fr", LaserScan, callback_laser, queue_size=1)
    rospy.Subscriber("/uwb", uwb, callback_uwb, queue_size=1)
    rospy.Subscriber("/received_messages", Frame, callback_gamma_v, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    listener()
