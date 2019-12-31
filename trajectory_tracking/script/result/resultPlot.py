#-----------将结果画图:一条正弦曲线------------

import numpy as np
import matplotlib.pyplot as plt
import math #数学计算相关包
from scipy import interpolate #平滑曲线
import time 

#图像滤波
def data_filter(data):
    #将数据从str转化成array
    np.set_printoptions(precision=4) #确定精确度为４
    dataFloat = data.astype('float64') #格式转换

    #数据筛选
    threhold = 60 #设定阈值
    dataFloatFilter = dataFloat[0:threhold,:]

    print(dataFloatFilter[:,0])
    print(dataFloatFilter[:,1])

    return dataFloatFilter 

#计算实际轨迹和理想轨迹的误差
def error(dataFloat):
    x_real = dataFloat[:,0] #取x轴数值
    y_real = dataFloat[:,1] #取y轴数值

    lenth = len(dataFloat[:,0]) #取x轴长度
    y_desire = np.zeros(lenth) #取y轴长度

    error = np.zeros(lenth) #误差向量
    error_abs = np.zeros(lenth) #误差向量
    
    for num in range(lenth):#计算对应误差
        #计算理想距离
        y_desire[num] = 3*math.sin(x_real[num]*np.pi/8) # list向量

        #计算误差
        error[num] = y_desire[num] - y_real[num]

        #计算绝对误差
        error_abs[num] = abs(error[num])

    print("error")
    print(error)

    return error,error_abs

#生成利萨如曲线（range_x=ｘ轴的范围，range_y=y轴的范围,步长＝step)
def lisajous(range_x,range_y,step): 
    theta = np.arange(0,2*np.pi,step*np.pi)

    position_x = range_x*np.sin(theta)

    position_y = range_y*np.sin(2*theta)

    print(position_x)
    print(position_y)

    #创建图像
    plt.figure(1)

    plt.plot(position_x,position_y)

    plt.show()

    return position_x,position_y

def result_plot(data):  
    #数据预处理
    dataFloat = data_filter(data) 
    
    #创建图像
    #plt.figure(figsize=(50,10))
    plt.figure(1)

    #-----------将实际&理想轨迹画图-----------
    #创建子图像
    plt.subplot(211)

    #定义横纵坐标
    plt.xlabel("position-X")
    plt.ylabel("position-Y")
    
    #设置横纵坐标范围
    plt.xlim((-4, 5))
    plt.ylim((-4, 4))

    #----------画图--理想轨迹-----------
    #理想轨迹生成
    x=np.arange(4,-3,-0.1)
    y=np.zeros(len(x))

    #插值取目标点
    for num in range(len(x)):
        y[num] = 3*math.sin(x[num]*np.pi/8) # list向量

    #画图
    plt.plot(ｘ,y)

    #-----------实际轨迹平滑-----------
    #画图－样条平滑一下
    f = interpolate.interp1d(dataFloat[:,0], dataFloat[:,1], kind='slinear') 

    x_interpolate =np.arange(4,-3,-0.005)

    y_interpolate = f(x_interpolate)

    print("x_interpolate")
    print(x_interpolate)

    print("y_interpolate")
    print(y_interpolate)

    print("lenth")
    print(len(y_interpolate))


    #1)一次性画图
    plt.plot(x_interpolate,y_interpolate)


    # #2)延迟画图
    # for num in range (len(x_interpolate)):
    #     plt.scatter(x_interpolate[num],y_interpolate[num],s=1,c='r')
    #     plt.pause(0.0001)


#-----------将误差画图-----------

    #-----------1.计算实际误差-----------
    error_x,error_abs = error(dataFloat)

    sum = 0

    #计算总误差
    for num in range(len(error_abs)):
        sum = sum + error_abs[num]
    
    #显示总误差＆平均误差
    print("总误差为")
    print(sum)

    print("平均误差为")
    print(sum/len(error_abs))
    
    #-----------2.将误差画图-----------
    #创建子图像
    plt.subplot(212)

    #定义横纵坐标
    plt.xlabel("step")
    plt.ylabel("error")
    
    #设置横纵坐标范围
    plt.xlim((0, 60))
    plt.ylim((0, 2))

    #计算实际轨迹和理想轨迹的误差
    error_x,error_abs = error(dataFloat)

    time = np.linspace(0,60,len(error_abs))

    #1.一次性画图
    plt.plot(time,error_abs)

    plt.show()


#载入txt文件
data = np.loadtxt('/home/qi/catkin_ws/src/amphibious_robot/trajectory_tracking/script/result/result/success_4_19.txt',dtype='str',skiprows=1,delimiter=",")

#取横纵坐标
data_position = data[:,[2,3]]

#画图
result_plot(data_position)



