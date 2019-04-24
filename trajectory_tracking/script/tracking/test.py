import rospy
import numpy as np
import math 
import matplotlib.pyplot as plt #画图工具


def lisajous(range_x,range_y,step): 
    theta = np.arange(0,2*np.pi,step*np.pi)

    position_x = -range_x*np.sin(theta)

    position_y = -range_y*np.sin(2*theta)

    print(position_x)
    print(position_y)

    #创建图像
    plt.figure(1)

    plt.plot(position_x,position_y)

    plt.show()

    return position_x,position_y

def doubleSinCurve(lineStep):
    #------------创建四条曲线------------
    #第一条正弦曲线
    sinOne_x=np.arange(4,-4,-0.5)
    sinOne_y=np.zeros(len(sinOne_x))

    #插值取目标点
    for num in range(len(sinOne_x)):
        sinOne_y[num] = 3*math.sin(sinOne_x[num]*np.pi/8) # list向量

    #第一条直线
    lineOne_y = np.arange(-3,3,lineStep)
    lineOne_x = -4*np.ones(len(lineOne_y))

    #第二条正弦曲线
    sinTwo_x=np.arange(-4,4,0.5)
    sinTwo_y=np.zeros(len(sinTwo_x)) 

    for num in range(len(sinTwo_x)):
        sinTwo_y[num] = -3*math.sin(sinTwo_x[num]*np.pi/8) # list向量

    #第二条直线
    lineTwo_y = np.arange(-3,3,lineStep)
    lineTwo_x = 4*np.ones(len(lineOne_y))

    #------------将四条曲线拼接在一起------------
    #创建x 
    x = np.append(sinOne_x,lineOne_x)
    x = np.append(x,sinTwo_x)
    x = np.append(x,lineTwo_x)

    #创建y
    y = np.append(sinOne_y,lineOne_y)
    y = np.append(y,sinTwo_y)
    y = np.append(y,lineTwo_y)

    return x,y


x,y = doubleSinCurve(0.5)

#创建图像
plt.figure(1)

plt.plot(x,y)

plt.show()