#-----------将结果画图
import numpy as np
import matplotlib.pyplot as plt
import math #数学计算相关包

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

def result_plot(data):  
    #数据预处理
    dataFloat = data_filter(data) 
    
    #创建图像
    plt.figure(1)

    #-----------将实际轨迹画图-----------
    #创建子图像
    plt.subplot(211)

    #定义横纵坐标
    plt.xlabel("position-X")
    plt.ylabel("position-Y")
    
    # #设置横纵坐标范围
    # plt.xlim((1, 5))
    # plt.ylim((1, 5))

    #画图
    plt.plot(dataFloat[:,0],dataFloat[:,1])

    #-----------将理想轨迹画图-----------
    # #创建子图像
    # plt.subplot(212)

    # #定义横纵坐标
    # plt.xlabel("理想轨迹－X")
    # plt.ylabel("理想轨迹－Y")
    
    # #设置横纵坐标范围
    # plt.xlim((1, 5))
    # plt.ylim((1, 5))

    #理想轨迹生成
    x=np.arange(4,0,-0.5)
    y=np.zeros(len(x))

    #插值取目标点
    for num in range(len(x)):
        y[num] = 3*math.sin(x[num]*np.pi/8) # list向量

    #画图
    plt.plot(ｘ,y)

    plt.show()

#载入txt文件
data = np.loadtxt('/home/qi/catkin_ws/src/amphibious_robot/trajectory_tracking/script/result/result/result_4_19.txt',dtype='str',skiprows=1,delimiter=",")

#取横纵坐标
data_position = data[:,[2,3]]

#画图
result_plot(data_position)

