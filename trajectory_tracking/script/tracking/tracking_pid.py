#coding=utf-8
import rospy #ros相关

from gazebo_msgs.msg import ModelStates #状态量数据订阅
from std_msgs.msg import Int16 #反馈需要
from sensor_msgs.msg import Imu #导入imu数据类型
from geometry_msgs.msg import Twist #控制命令需要
from std_msgs.msg import Float64MultiArray #自己的话题transformation 

import math 
import numpy as np
import time 

from tf_my import euler_from_quaternion, quaternion_from_euler #将接收到的四元数转化为转角信息

import matplotlib.pyplot as plt #画图工具

#全局变量赋值
is_done=Int16()
is_done.data=0

state=Float64MultiArray()#只有xyz
state.data=[0,0,0]

count = 12

class tracking_sin():
    def __init__(self):
        #---------初始化-----------
        #------状态量------
        self.quaternion = np.zeros(4) #四元数
        self.euler_angle = np.zeros(3) #转角信息
        self.positions = np.zeros(3) #位置信息
        self.point_to_target = np.zeros(2) #当前点和目标点之间的向量
        self.yaw_point_to_target = 0 #当前点和目标点向量之间的夹角

        self.distance = 0 #理想点和目标点之间的距离

        #------状态量------
        self.forward_flag = 0 #直行的flag 

        #---------创造期望轨迹-----------
        self.x,self.y = self.curve_generate(0.5)

        print(self.x)
        print(self.y)

        self.count = 2

        #---------跟踪算法-----------
        #跟踪
        rospy.init_node("tracking_pid",anonymous=True) #初始化节点

        #发布位姿信息
        self.pub = rospy.Publisher("transformation",Float64MultiArray,queue_size=3)

        #订阅位姿信息
        rospy.Subscriber("imu", Imu, self.callback)  #不停触发执行实时轨迹跟踪主函数

        rospy.Subscriber("is_done", Int16, self.callback_2) #订阅反馈：静止发送is_done=1，运动不发送任何消息

        rospy.Subscriber("gazebo/model_states", ModelStates, self.callback_3)#获得实时角度和位置

    #定义第一个回调函数 (注意可能会求反的)
    def callback(self,data):
        global count #声明全局变量

        print("目标点")
        # print(self.count)
        print(count)

        print("理想位置")
        # print(self.x[self.count],self.y[self.count])
        print(self.x[count],self.y[count])

        print("实际位置")
        print(self.positions[0:2])

        #计算实际位置和理想点之间的向量 :目标点-当前点
        # self.point_to_target[0] = self.x[self.count]-self.positions[0] #方向向量x
        # self.point_to_target[1] = self.y[self.count]-self.positions[1] #方向向量y

        self.point_to_target[0] = self.x[count]-self.positions[0] #方向向量x
        self.point_to_target[1] = self.y[count]-self.positions[1] #方向向量y

        #求该方向向量的yaw角
        self.yaw_point_to_target = self.angle(self.point_to_target[0],self.point_to_target[1])

        print("理想yaw角")
        print(self.yaw_point_to_target*180/np.pi)

        print("实际yaw角")
        print(self.euler_angle[2]*180/np.pi)

        #计算理想点和实际点之间的距离
        self.distance = self.distance_point_to_target(self.point_to_target[0],self.point_to_target[1])

        print("distance")
        print(self.distance)

        if self.distance > 0.15 : #误差大于0.1米
            print("缩小误差")
            #转动到期望的角度 
            if not (self.yaw_point_to_target - np.pi/5 < self.euler_angle[2] < self.yaw_point_to_target + np.pi/5 ): #没在规定角度之内
                print("转动到期望的角度")
                self.turn_to_target_angle(self.yaw_point_to_target)

            else:
                print("直行")
                self.go_forward() 
                time.sleep(2)

        else:
            print("没有误差")

            # if self.count <= (len(self.x)-1) :
            #     self.count = self.count+1 #目标点换成下一个点
            # else :
            #     print("最后一个点")
            #     self.count = 0

            if count >=55 :
                count = 0
            else :
                count = count+1 #目标点换成下一个点
    
    #生成双正弦曲线（lineStep 步长）
    def curve_generate(self,lineStep):
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

        #创建图像
        plt.figure(1)

        plt.plot(x,y)

        plt.show()

        return x,y

    #计算实时点和目标点之间的距离
    def distance_point_to_target(self,distance_x,distance_y):
        return np.sqrt(np.square(distance_x)+np.square(distance_y))

    #求实时和期望的夹角
    def angle(self,point_x,point_y):
        x=np.array([-1,0])
        y=np.array([point_x,point_y])
        # 两个向量
        Lx=np.sqrt(x.dot(x))
        Ly=np.sqrt(y.dot(y))
        #相当于勾股定理，求得斜线的长度
        cos_angle=x.dot(y)/(Lx*Ly)
        #求得cos_sita的值再反过来计算，绝对长度乘以cos角度为矢量长度，初中知识。。
        # print(cos_angle)
        angle=np.arccos(cos_angle)
        angle2=angle*360/2/np.pi
        #变为角度
        # print(angle2)

        #判断角度的正负
        if point_y > 0: 
            return -angle
        else :
            return angle 
        #x.dot(y) =  y=∑(ai*bi)

    #运动---转动至期望角度
    def turn_to_target_angle(self,object_angle):
        # #初始化
        # is_done.data = 0

        #调试
        print("理想角度是")
        print(object_angle)

        print("实际角度是")
        print(self.euler_angle[2])

        #将角度全部变为正数
        object_angle_rect=object_angle+np.pi
        euler_angle_rect=self.euler_angle[2]+np.pi

        #目标角度和实际角度做差（取绝对值）
        error_angle = np.abs((object_angle_rect-euler_angle_rect))

        #如果有误差转动，没误差不转动
        if error_angle>np.pi:
            #error_angle_rect = error_angle-np.pi
            error_angle_rect = 2*np.pi-error_angle
        else:
            error_angle_rect = error_angle

        print("两个角度之间的误差是")
        print(error_angle_rect)

        #误差要是大于pi/15，就转动，要是小于pi/15，就不转动
        if not error_angle_rect < np.pi/15:
            if error_angle<=np.pi:
                if euler_angle_rect<object_angle_rect:#<target 逆时针
                    if(is_done.data==1):
                        #print("静止中")

                        #声明变量
                        u = np.zeros(2) #控制量声明

                        #赋值角速度
                        u[0] = 0
                        u[1] = -1
                        
                        #发布线速度消息:发送数次
                        for num in range(5):
                            self.data_publish(u)

                        #暂停一秒，让机器人充分运动
                        time.sleep(1)
                        
                        #归零
                        is_done.data = 0

                        print("逆时针旋转")
                
                    elif(is_done.data == 0):
                        print("运动中")
                if euler_angle_rect>=object_angle_rect:#>target 顺时针
                    if(is_done.data==1):
                        #print("静止中")

                        #声明变量
                        u = np.zeros(2) #控制量声明

                        #赋值角速度
                        u[0] = 0
                        u[1] = 1
                        
                        #发布线速度消息:发送数次
                        for num in range(5):
                            self.data_publish(u)

                        #暂停一秒，让机器人充分运动
                        time.sleep(1)
                        
                        #归零
                        is_done.data = 0

                        print("顺时针旋转")
                
                    elif(is_done.data == 0):
                        print("运动中")

            if error_angle>np.pi:
                if euler_angle_rect>=object_angle_rect:#>target 逆时针
                    if(is_done.data==1):
                        #print("静止中")

                        #声明变量
                        u = np.zeros(2) #控制量声明

                        #赋值角速度
                        u[0] = 0
                        u[1] = -1
                        
                        #发布线速度消息:发送数次
                        for num in range(5):
                            self.data_publish(u)

                        #暂停一秒，让机器人充分运动
                        time.sleep(1)
                        
                        #归零
                        is_done.data = 0

                        print("逆时针旋转")
                
                    elif(is_done.data == 0):
                        print("运动中")
                if euler_angle_rect<object_angle_rect:#<target 顺时针
                    if(is_done.data==1):
                        #print("静止中")

                        #声明变量
                        u = np.zeros(2) #控制量声明

                        #赋值角速度
                        u[0] = 0
                        u[1] = 1
                        
                        #发布线速度消息:发送数次
                        for num in range(5):
                            self.data_publish(u)

                        #暂停一秒，让机器人充分运动
                        time.sleep(1)
                        
                        #归零
                        is_done.data = 0

                        print("顺时针旋转")
                
                    elif(is_done.data == 0):
                        print("运动中")
        else:
            print("没有误差hhhhhhh")

            # #已经控制完一轮了
            # self.control_once_flag = 0

            #模式切换成功归零
            
            #表明转换到目标角度之间成功
            self.turn_to_target_angle_success = 1
        
        # #执行控制 self.euler_angle[2] = yaw (实时的)
        # if not (object_angle - np.pi/15) < self.euler_angle[2] < (object_angle + np.pi/15): 
        #     if (self.euler_angle[2]<object_angle): #逆时针转动
        #         if(is_done.data==1):
        #             #print("静止中")

        #             #声明变量
        #             u = np.zeros(2) #控制量声明

        #             #赋值角速度
        #             u[0] = 0
        #             u[1] = -1
                    
        #             #发布线速度消息:发送数次
        #             for num in range(5):
        #                 self.data_publish(u)

        #             #暂停一秒，让机器人充分运动
        #             time.sleep(1)
                    
        #             #归零
        #             is_done.data = 0

        #             print("逆时针旋转")
            
        #         elif(is_done.data == 0):
        #             print("运动中")

        #     elif(self.euler_angle[2]>=object_angle): #顺时针转动
        #         if(is_done.data==1):
        #             #print("静止中")

        #             #声明变量
        #             u = np.zeros(2) #控制量声明

        #             #赋值角速度
        #             u[0] = 0
        #             u[1] = 1
                    
        #             #发布线速度消息:发送数次
        #             for num in range(5):
        #                 self.data_publish(u)

        #             #暂停一秒，让机器人充分运动
        #             time.sleep(1)

        #             #归零
        #             is_done.data = 0

        #             print("顺时针旋转")
            
        #         elif(is_done.data == 0):
        #             print("运动中")

        # else:
        #     print("没有误差")

        #     # #已经控制完一轮了
        #     # self.control_once_flag = 0

        #     #模式切换成功归零
        #     self.count = 0
            
        #     #表明转换到目标角度之间成功
        #     self.turn_to_target_angle_success = 1

    #运动---前进
    def go_forward(self):
        while(1):
            #静止时候赋值运动
            if(self.forward_flag == 0):
                if(is_done.data==1):
                    #声明变量
                    u = np.zeros(2) #控制量声明

                    #赋值线速度
                    u[0] = 1
                    u[1] = 0
                    
                    #发布线速度消息:发送数次
                    for num in range(5):
                        self.data_publish(u)

                    #调试
                    print("直行")
                    
                    self.forward_flag =1 

            #运动完，归零退出循环
            elif(self.forward_flag == 1):
                #归零
                self.forward_flag =0 

                is_done.data = 0
                
                print("退出循环")

                #跳出循环
                break 

    #定义第二个回调函数:实现对机器人动静状态的实时接受（静止is_done=1,运动is_done=0)
    def callback_2(self,data): 
        #接受数据
        is_done.data= data.data

    #定义第三个回调函数
    #获得实时角度和位置
    def callback_3(self,data):
        #获得实际位姿
        pose_orientation = data.pose[1] #这个是实时位姿，数据类型是pose,包括position 和orientation 

        #将位姿转化并储存  
        self.position(pose_orientation) #位置
        self.orientation(pose_orientation) #姿态

        #赋值
        state.data[0]=data.pose[1].position.x
        state.data[1]=data.pose[1].position.y
        state.data[2]=data.pose[1].position.z

    #获取位置的函数:位置被实时保存在了self.position里面
    def position(self,data):
        #获取此时的position
        self.positions[0] = data.position.x #x坐标
        self.positions[1] = data.position.y #y坐标
        self.positions[2] = data.position.z #z坐标

        # #显示此时的position
        # print("x",self.positions[0])
        # print("y",self.positions[1])
        # print("z",self.positions[2])

    #获取姿态的函数：位置被实时保存在了self.orientation里面
    def orientation(self,data):
        #获取此时的orientation
        self.quaternion[0] = data.orientation.x #四元数
        self.quaternion[1] = data.orientation.y #四元数
        self.quaternion[2] = data.orientation.z #四元数
        self.quaternion[3] = data.orientation.w #四元数
        
        # #显示此时的姿态对应的四元素
        # print("x",self.quaternion[0])
        # print("y",self.quaternion[1])
        # print("z",self.quaternion[2])
        # print("w",self.quaternion[2])

        #将四元素转化成欧拉角
        self.euler_angle = euler_from_quaternion(self.quaternion)

    #定义发送数据的函数:只发一次（可以考虑发很多次，最终要看效果如何）
    def data_publish(self,u):
        #初始化node
        
        #初始化publisher
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) # topic和消息类型需要和肖岸星商量确定一下 
        
        #初始化twist
        twist = Twist()

        # 给twist赋值
        twist.linear.x = u[0] #线速度
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0 #角速度
        twist.angular.y = 0.0
        twist.angular.z = u[1]

        #调试代码
        # print("twist")
        # print(twist)

        #发送twist
        pub.publish(twist) #将数据发送出去

#创建主函数
if __name__ == '__main__':
    #初始化对象
    dataset = tracking_sin()

    #发布消息
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        dataset.pub.publish(state)
        rate.sleep()

    #监控pub和sub
    rospy.spin()
