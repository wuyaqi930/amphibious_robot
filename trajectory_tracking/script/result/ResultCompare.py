#-----------将简单的和复杂的作对比------------
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
    # #生命图像
    # plt.figure(2)

    #PID 误差
    if typeOfCurve == 1 :
        #声明横纵坐标
        plt.xlabel("step")
        plt.ylabel("error")

        #设置横纵坐标范围
        # plt.xlim((0, len(error)+1))
        plt.ylim((0, 1))

        step = np.arange(1,len(error)+1,1)
        
        plt.plot(step,error,'r',label='Line Curve 1')

        plt.legend()
    # 误差
    else:
        #声明横纵坐标
        plt.xlabel("step")
        plt.ylabel("error")

        #设置横纵坐标范围
        # plt.xlim((0, len(error)+1))
        plt.ylim((0, 1))

        step = np.arange(1,len(error)+1,1)

        plt.plot(step,error,'r',label='Line Curve 2')

        plt.legend()

    plt.title('Error of Neural Networks Tracking Method') 


    # plt.show()

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
    # x=np.arange(max,min,-0.1)
    # y=np.zeros(len(x))
    x=np.arange(4,-4.1,-0.1)
    y=np.zeros(len(x))

    #插值取目标点
    if typeOfCurve == 1:#第一条正弦
        for num in range(len(x)):
            y[num] = 3*math.sin(x[num]*np.pi/8) # list向量

    elif typeOfCurve == 2:#第二条正弦
        for num in range(len(x)):
            y[num] = -3*math.sin(x[num]*np.pi/8) # list向量

    #画图
    plt.plot(ｘ,y,color='black')

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
    # plt.plot(x_interpolate,y_interpolate,color='blue',linestyle='dashed')
    # plt.plot(x_interpolate,y_interpolate,color='blue')
    plt.plot(x_interpolate,y_interpolate)

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
    plt.plot(line_x,line_y,color='black')

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
    #plt.plot(x_interpolate,y_interpolate,color='blue',linestyle='dashed')
    plt.plot(x_interpolate,y_interpolate,color='blue')

    #plt.show()

#画任意一段曲线
def drawPoint(data):
    # #声明图像
    # plt.figure(1)

    #声明横纵坐标
    plt.xlabel("X")
    plt.ylabel("Y")

    #设置横纵坐标范围
    plt.xlim((-5, 5))
    plt.ylim((-5, 5))
    # plt.plot(data[:,0],data[:,1],color='blue', marker='s', linestyle='dashed',markeredgecolor='r',markerfacecolor='r',linewidth=1, markersize=3)
    plt.plot(data[:,0],data[:,1],'ro',markersize=3)

    # plt.show()

#画两点之间的连线
def drawPointLine(data):
    # #声明图像
    # plt.figure(1)

    #声明横纵坐标
    plt.xlabel("X")
    plt.ylabel("Y")

    #设置横纵坐标范围
    plt.xlim((-5, 5))
    plt.ylim((-5, 5))
    
    # plt.plot(data[:,0],data[:,1],color='blue', marker='s', linestyle='dashed',markeredgecolor='r',markerfacecolor='r',linewidth=1, markersize=3)
    plt.plot(data[:,0],data[:,1],'bo-',markersize=3,markeredgecolor='r',markerfacecolor='r')

    # plt.show()

#画两曲线之间的连线
def BetweenCurve(num):
    if num == 1 :
        drawPointLine(data_pid[1637:1639,:])

        #2-3连接处
        point = []
        point.append(list(data_pid[1638]))
        point.append(list(data_pid[1675]))
        point = np.array(point)

        drawPointLine(point)

        #3-4连接处
        point = []
        point.append(list(data_pid[2130]))
        point.append(list(data_pid[2131]))
        point = np.array(point)

        drawPointLine(point)

        #4-1连接处
        point = []
        point.append(list(data_pid[2180]))
        point.append(list(data_pid[1469]))
        point = np.array(point)

        drawPointLine(point)

    else:
        #1-2连接处
        point = []
        point.append(list(data_network[1932]))
        point.append(list(data_network[1928]))
        point = np.array(point)

        drawPointLine(point)

        #2-3连接处
        point = []
        point.append(list(data_network[1949]))
        point.append(list(data_network[1950]))
        point = np.array(point)

        drawPointLine(point)

        #3-4连接处
        point = []
        point.append(list(data_network[1984]))
        point.append(list(data_network[1985]))
        point = np.array(point)

        drawPointLine(point)

        #3-4连接处
        point = []
        point.append(list(data_network[2007]))
        point.append(list(data_network[1885]))
        point = np.array(point)

        drawPointLine(point)

def CompareSinCurve(dataOfCurve,typeOfCurve,number):
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
    # x=np.arange(max,min,-0.1)
    # y=np.zeros(len(x))
    x=np.arange(4,-4.1,-0.1)
    y=np.zeros(len(x))

    #插值取目标点
    if typeOfCurve == 1:#第一条正弦
        for num in range(len(x)):
            y[num] = 3*math.sin(x[num]*np.pi/8) # list向量

    elif typeOfCurve == 2:#第二条正弦
        for num in range(len(x)):
            y[num] = -3*math.sin(x[num]*np.pi/8) # list向量

    #画图
    plt.plot(ｘ,y,color='black')

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


    print("num")
    print(number)

    #显示图例
    if number == 1:
        plt.plot(x_interpolate,y_interpolate,label='tracking_normal')
        print("ok")
    else:
        plt.plot(x_interpolate,y_interpolate,label='tracking_networks')

    plt.legend() 

def CompareLineCurve(dataOfCurve,typeOfCurve,lineStep,number):
    #----------画图--理想轨迹-----------
    #声明图像
    #plt.figure(1)

    #设置横纵坐标范围
    plt.xlim((-5, 5))
    plt.ylim((-4, 4))

    #理想轨迹生成
    if typeOfCurve==1: #第一条直线
        line_y = np.arange(-3,3,lineStep)
        line_x = -4*np.ones(len(line_y))

    else: #第二条直线
        line_y = np.arange(-3,3,lineStep)
        line_x = 4*np.ones(len(line_y))

    #画图
    plt.plot(line_x,line_y,color='black')

    # plt.plot(dataOfCurve[:,0],dataOfCurve[:,1])

    #----------画图--实际轨迹-----------
    #求y的最小值和最大值
    min = np.min(dataOfCurve[:,1])
    max = np.max(dataOfCurve[:,1])

    #画图－样条平滑一下
    f = interpolate.interp1d(dataOfCurve[:,1], dataOfCurve[:,0], kind='slinear')

    y_interpolate =np.arange(min,max,0.005)

    x_interpolate = f(y_interpolate)

    #显示图例
    if number == 1:
        plt.plot(x_interpolate,y_interpolate,label='tracking_normal')
    else:
        plt.plot(x_interpolate,y_interpolate,label='tracking_networks')

    plt.legend() 

#求误差最大值、最小值、平均值、优秀值、步数
def ComputeError(dataOfError):
    lenth = len(dataOfError)

    sum = 0 #总和
    max = 0 #最大值
    min = 10000 #最小值
    average = 0 #平均值

    threhold = 0.2 #阈值
    perfectCount = 0 #优秀的误差
    step  = len(dataOfError) #总步数

    for num in range(step):
        #求平均值
        sum += dataOfError[num]

        #求最大值
        if dataOfError[num] > max:
            max = dataOfError[num]

        #求最小值
        if dataOfError[num] < min:
            min = dataOfError[num]

        #求优秀值比例
        if dataOfError[num] <= threhold:
            perfectCount = perfectCount+1

    #计算出比例
    average = sum/step #计算平均值
    perfectCount = perfectCount/step #计算优秀率

    return average,max,min,step,perfectCount

#读取数据
data_network = np.loadtxt('/home/qi/catkin_ws/src/amphibious_robot/trajectory_tracking/script/result/result/txt/success_4_29.txt',dtype='str',skiprows=1,delimiter=",")
data_pid = np.loadtxt('/home/qi/catkin_ws/src/amphibious_robot/trajectory_tracking/script/result/result/txt/5_22_0.1_pi_5.txt',dtype='str',skiprows=1,delimiter=",")

#取横纵坐标
data_network = data_network[:,[2,3]]
data_pid = data_pid[:,[2,3]]

#滤波
data_network = data_filter(data_network,7855)
data_pid = data_filter(data_pid,3971)


#-----------画整体轨迹图-----------
plt.figure(1)

# #-----------描点-----------
# # 第一条正弦
# drawPoint(data_pid[1465:1570,:])

# # 第一条直线
# drawPoint(data_pid[1575:1602,:])
# drawPoint(data_pid[1605:1639,:])

# #第二条正弦
# drawPoint(data_pid[1675:1698,:])
# drawPoint(data_pid[1702:1750,:])
# drawPoint(data_pid[2005:2130,:])

# # 第二条直线
# drawPoint(data_pid[2130:2181,:])

# #-----------画pid轨迹-----------
# # 第一条正弦
# drawSinCurve(data_pid[1465:1570,:],1)

# # 第一条直线
# drawLineCurve(data_pid[1575:1638,:],1,0.05)

# # 第二条正弦
# drawSinCurve(data_pid[1675:2132,:],2)

# # 第二条直线
# drawLineCurve(data_pid[2131:2181,:],2,0.05)

# # 补充两点之间的连线
# BetweenCurve(1)


# #-----------描点----------
# drawPoint(data_network[1885:2008,:])

# #-----------画network轨迹-----------
# # 第一种
# # 第一条正弦
# drawSinCurve(data_network[1885:1931,:],1)

# # 第一条直线
# drawLineCurve(data_network[1931:1950,:],1,0.05)

# # 第二条正弦
# drawSinCurve(data_network[1950:1985,:],2)

# # 第二条直线
# drawLineCurve(data_network[1985:2008,:],2,0.05)

# #-----------补充两点之间的连线-----------
# BetweenCurve(2)

#-----------画误差图-----------
# 第一条正弦
# error_pid = errorSinCurve(data_pid[1465:1570,:],1)
# error_networks = errorSinCurve(data_network[1885:1931,:],1)

# 第一条直线
# error_pid = errorLineCurve(data_pid[1575:1638,:],1)
# error_networks = errorLineCurve(data_network[1931:1950,:],1)

# 第二条正弦
# error_pid = errorSinCurve(data_pid[1675:2132,:],2)
# error_networks = errorSinCurve(data_network[1950:1985,:],2)

# 第二条直线
# error_pid = errorLineCurve(data_pid[2131:2181,:],2)
error_networks = errorLineCurve(data_network[1985:2008,:],2)

# drawError(error_pid,2)
# drawError(error_networks,2)

# 计算实际的误差
average,max,min,step,perfectCount = ComputeError(error_networks )

print("平均数")
print(average)

print("最大值")
print(max)

print("最小值")
print(min)

print("总步数")
print(step)

print("优秀率")
print(perfectCount)
#-----------画轨迹对比图-----------
# #正弦轨迹
# CompareSinCurve(data_pid[1675:2132,:],2,1)
# CompareSinCurve(data_network[1950:1985,:],2,2)

#直线轨迹
# #第一条
# CompareLineCurve(data_pid[1575:1638,:],1,0.05,1)
# CompareLineCurve(data_network[1931:1950,:],1,0.05,2)

# #第二条
# CompareLineCurve(data_pid[2131:2181,:],2,0.05,1)
# CompareLineCurve(data_network[1985:2008,:],2,0.05,2)

plt.show()


