#-----------将结果画图:两条正弦曲线 两条直线------------

import numpy as np
import matplotlib.pyplot as plt
import math #数学计算相关包
from scipy import interpolate #平滑曲线
import time


#图像滤波
def data_filter(data,numberOfData):
    #将数据从str转化成array
    np.set_printoptions(precision=4) #确定精确度为４
    dataFloat = data.astype('float64') #格式转换

    #数据筛选
    #threhold = 60 #设定阈值
    threhold = numberOfData #设定阈值
    dataFloatFilter = dataFloat[0:threhold,:]

    # print(dataFloatFilter[:,0])
    # print(dataFloatFilter[:,1])

    return dataFloatFilter

#正弦误差 numberOfCurve 1:第一条正弦曲线 2:第二条正弦曲线
def errorSinCurve(dataSin,numberOfCurve):

    print("dataSin")
    print(dataSin)

    error = np.zeros(len(dataSin[:,0]))

    if numberOfCurve == 1:
        for num in range(len(dataSin[:,0])):
            error[num] = abs(dataSin[num,1]-3*math.sin(dataSin[num,0]*np.pi/8)) #第一条正弦曲线

        # step = np.arange(1,len(dataSin[:,0])+1,1)

        error_average = np.sum(abs(error))/45

        print("error_average")
        print(error_average)

    elif numberOfCurve == 2:
        for num in range(len(dataSin[:,0])):
            error[num] = abs(dataSin[num,1]+3*math.sin(dataSin[num,0]*np.pi/8)) #第二条正弦曲线

        # step = np.arange(1,len(dataSin[:,0])+1,1)

        error_average = np.sum(abs(error))/45

        print("error_average")
        print(error_average)

    return error

#直线误差 1:第一条直线 2:第二条直线
def errorLineCurve(dataLine,numberOfCurve):

    error = np.zeros(len(dataLine[:,0]))

    if numberOfCurve == 1:
        for num in range(len(dataLine[:,0])):
            error[num] = abs(dataLine[num,0]+4) #第一条直线

        # step = np.arange(1,len(dataSin[:,0])+1,1)

        error_average = np.sum(abs(error))/len(dataLine[:,0])

        print("error_average")
        print(error_average)

    elif numberOfCurve == 2:
        for num in range(len(dataLine[:,0])):
            error[num] = abs(dataLine[num,0]-4) #第一条直线

        # step = np.arange(1,len(dataSin[:,0])+1,1)

        error_average = np.sum(abs(error))/len(dataLine[:,0])

        print("error_average")
        print(error_average)

    return error

#画误差
def drawError(error,typeOfCurve):
    #生命图像
    plt.figure(2)

    #声明横纵坐标
    plt.xlabel("step")
    plt.ylabel("error")

    #设置横纵坐标范围
    plt.xlim((0, len(error)+1))
    plt.ylim((0, 1))

    step = np.arange(1,len(error)+1,1)

    plt.plot(step,error)

    plt.show()

#画正弦曲线
def drawSinCurve(dataOfCurve,typeOfCurve):
    #----------画图--理想轨迹-----------
    #声明图像
    #plt.figure(1)

    #求x的最小值和最大值
    min = np.min(dataOfCurve[:,0])
    max = np.max(dataOfCurve[:,0])

    #设置横纵坐标范围
    plt.xlim((-5, 5))
    plt.ylim((-5, 5))

    #理想轨迹生成
    x=np.arange(max,min,-0.1)
    y=np.zeros(len(x))

    #插值取目标点
    if typeOfCurve == 1:#第一条正弦
        for num in range(len(x)):
            y[num] = 3*math.sin(x[num]*np.pi/8) # list向量

    elif typeOfCurve == 2:#第二条正弦
        for num in range(len(x)):
            y[num] = -3*math.sin(x[num]*np.pi/8) # list向量

    #画图
    plt.plot(ｘ,y)

    #-----------实际轨迹平滑-----------
    # #求y的最小值和最大值
    # min = np.min(dataOfCurve[:,0])
    # max = np.max(dataOfCurve[:,0])

    #画图－样条平滑一下
    f = interpolate.interp1d(dataOfCurve[:,0], dataOfCurve[:,1], kind='slinear')

    if typeOfCurve==1:
        x_interpolate =np.arange(max,min,-0.005)
    elif typeOfCurve==2:
        x_interpolate =np.arange(min,max,0.005)

    y_interpolate = f(x_interpolate)

    print("x_interpolate")
    print(x_interpolate)

    print("y_interpolate")
    print(y_interpolate)

    #1)一次性画图
    plt.plot(x_interpolate,y_interpolate)

    #plt.show()

#画直线
def drawLineCurve(dataOfCurve,typeOfCurve,lineStep):
    #----------画图--理想轨迹-----------
    #声明图像
    #plt.figure(1)

    #设置横纵坐标范围
    plt.xlim((-5, 5))
    plt.ylim((-5, 5))

    #理想轨迹生成
    if typeOfCurve==1: #第一条直线
        line_y = np.arange(-3,3,lineStep)
        line_x = -4*np.ones(len(line_y))

    else: #第二条直线
        line_y = np.arange(-3,3,lineStep)
        line_x = 4*np.ones(len(line_y))

    #画图
    plt.plot(line_x,line_y)

    # plt.plot(dataOfCurve[:,0],dataOfCurve[:,1])

    #----------画图--实际轨迹-----------
    #求y的最小值和最大值
    min = np.min(dataOfCurve[:,1])
    max = np.max(dataOfCurve[:,1])

    #画图－样条平滑一下
    f = interpolate.interp1d(dataOfCurve[:,1], dataOfCurve[:,0], kind='slinear')

    y_interpolate =np.arange(min,max,0.005)

    x_interpolate = f(y_interpolate)

    print("x_interpolate")
    print(x_interpolate)

    print("y_interpolate")
    print(y_interpolate)

    #1)一次性画图
    plt.plot(x_interpolate,y_interpolate)

    #plt.show()

#画任意一段曲线
def draw(data):
    #生命图像
    plt.figure(1)

    #声明横纵坐标
    plt.xlabel("X")
    plt.ylabel("Y")

    #设置横纵坐标范围
    plt.xlim((-5, 5))
    plt.ylim((-5, 5))

    plt.plot(data[:,0],data[:,1])

    plt.show()

#载入txt文件
# data = np.loadtxt('/home/qi/catkin_ws/src/amphibious_robot/trajectory_tracking/script/result/result/txt/5_22_0.1_pi_5.txt',dtype='str',skiprows=1,delimiter=",")
# data = np.loadtxt('/home/qi/catkin_ws/src/amphibious_robot/trajectory_tracking/script/result/result/txt/success_4_29.txt',dtype='str',skiprows=1,delimiter=",")
data = np.loadtxt('/home/qi/catkin_ws/src/amphibious_robot/trajectory_tracking/script/result/result/txt/tracking_pid_dnn_3.txt',dtype='str',skiprows=1,delimiter=",")


#取横纵坐标
data_position = data[:,[2,3]]

#滤波
# dataFloat = data_filter(data_position,3971)
# dataFloat = data_filter(data_position,7855)
dataFloat = data_filter(data_position,2818)


#-------------数据集划分-------------

# dis=0.25 angular=15/pi 
# 第一段曲线 540:675 540:580 580:605 605:647 647:675
# 第二段曲线 810:965 810:855 855:883 883:928 928:965

# dis=0.15 angular=10/pi 
# (3) 第一圈：0:150 第二圈 150:350 第三圈：350:565
# 取第二圈：1.150:200 2.200:245 3.245:325 4.325:350
# (2) 第一圈：0:185 
# (1) 第一圈：0:135 

# tracking_networks
# success_4_29.txt: 
# 所有圈数：0:160 160:310 310:450 450:605 605:740 740:890 890:1050 1050:1240
# 1240:1430 1430:1580 1580:1720 1720:1885 2335:2500 2335:2500 2500:2660
# 
# 好圈 ：1885:2010 ：1.1885:1930 2.1930:1950 3.1950:1980 4.1950:1985 5.2010
# 好圈：5435:5580 2010:2175 2175:2335
# 好圈：450:605 1580:1720

# dis=0.1 angular=5/pi
# 5_22_0.1_pi_5.txt
# 第一圈：350:910 
# 第二圈：910:1455
# 第三圈：1455:2180 1455:1610 1610:1675 1675:2130 2130:2180
# 第四圈：2180:2710

#pid+dnn时候
# tracking_pid_dnn_3.txt
# 圈数：647:770: 647:682 682:706 706:747 747:770




#-------------分别画出四条曲线-------------

plt.figure(1)

#画任意一段曲线
draw(dataFloat[747:770,:])


# # ----------pid时候的----------
# # 第一条正弦
# drawSinCurve(dataFloat[1455:1610,:],1)

# # 第一条直线
# drawLineCurve(dataFloat[1610:1675,:],1,0.05)

# #第二条正弦
# drawSinCurve(dataFloat[1675:2130,:],2)

# # 第二条直线
# drawLineCurve(dataFloat[2130:2180,:],2,0.05)

# # ----------network时候的----------
# # 第一条正弦
# drawSinCurve(dataFloat[1885:1930,:],1)

# # # 第一条直线
# drawLineCurve(dataFloat[1930:1950,:],1,0.05)

# # # #第二条正弦
# drawSinCurve(dataFloat[1950:1985,:],2)

# # # # 第二条直线
# drawLineCurve(dataFloat[1985:2010,:],2,0.05)

# ----------angal+networks时候的----------
# 第一条正弦
drawSinCurve(dataFloat[785:827,:],1)

# # 第一条直线
# drawLineCurve(dataFloat[1610:1675,:],1,0.05)

# #第二条正弦
# drawSinCurve(dataFloat[1675:2130,:],2)

# # 第二条直线
# drawLineCurve(dataFloat[2130:2180,:],2,0.05)


plt.show()

#-------------画出对应的误差-------------

error = errorSinCurve(dataFloat[785:827,:],1)
# error = errorLineCurve(dataFloat[1610:1675,:],1)

drawError(error,1)




