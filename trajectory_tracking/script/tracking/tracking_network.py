#coding=utf-8
#------------------导入官方package------------------
import rospy #ros相关

from gazebo_msgs.msg import ModelStates #状态量数据订阅
from std_msgs.msg import Int16 #反馈需要
from sensor_msgs.msg import Imu #导入imu数据类型
from geometry_msgs.msg import Twist #控制命令需要
from std_msgs.msg import Float64MultiArray #自己的话题transformation 
import matplotlib.pyplot as plt #画图工具

import math 
import numpy as np
import time 

from tf_my import euler_from_quaternion, quaternion_from_euler #将接收到的四元数转化为转角信息

#------------------导入自己package------------------
import optimize

#全局变量赋值
is_done=Int16()
is_done.data=0

state=Float64MultiArray()#只有xyz
state.data=[0,0,0]


class tracking_networks():
    def __init__(self):
        #---------初始化--------
        self.quaternion = np.zeros(4) #四元数
        self.euler_angle = np.zeros(3) #转角信息
        self.positions = np.zeros(3) #位置信息
        self.step = 2 #步长信息为５
        self.count = 0 #目标点的象征
        self.count_number = 0 #走过了的距离
        self.control_final = np.zeros((self.step,2))#最终控制序列

        self.x_init = np.zeros(3) #位置信息
        self.x_desire = np.zeros(3) #位置信息

        self.go_flag=0 #表示机器人运动第几步

        self.stepPer = 1

        self.point_to_target = np.zeros(2) #当前点和目标点之间的向量

        #初始化mpc
        self.mpc = optimize.MPC(self.x_init,self.x_desire) #初始化
        self.mpc.restore_params() #重新载入网络

#---------生成理想轨迹:正弦曲线---------
        # #self.x=np.arange(-4,4,0.5)
        # self.x=np.arange(4,-4,-0.5)
        # self.y=np.zeros(len(self.x))

        # #插值取目标点
        # for num in range(len(self.x)):
        #     self.y[num] = 3*math.sin(self.x[num]*np.pi/8) # list向量

        # self.count = 0 #第几个目标点

        # print(self.x)
        # print(self.y)

#---------生成理想轨迹：利萨如曲线---------
        #self.x,self.y = self.lisajous(4,4,0.05)
        self.x,self.y = self.curve_generate(0.5)

        print(self.x)
        print(self.y)

        self.count = 10#第几个目标点

        #---------跟踪算法-----------
        #跟踪
        rospy.init_node("tracking_networks",anonymous=True) #初始化节点

        #发布位姿信息
        self.pub = rospy.Publisher("transformation",Float64MultiArray,queue_size=3)

        #订阅位姿信息
        rospy.Subscriber("imu", Imu, self.callback)  #不停触发执行实时轨迹跟踪主函数

        rospy.Subscriber("is_done", Int16, self.callback_2) #订阅反馈：静止发送is_done=1，运动不发送任何消息

        rospy.Subscriber("gazebo/model_states", ModelStates, self.callback_3)#获得实时角度和位置

    #生成利萨如曲线（range_x=ｘ轴的范围，range_y=y轴的范围,步长＝step)
    def lisajous(self,range_x,range_y,step): 
        theta = np.arange(0,2*np.pi,step*np.pi)

        position_x = -range_x*np.sin(theta)

        position_y = -range_y*np.sin(2*theta)

        print(position_x)
        print(position_y)

        # #创建图像
        # plt.figure(1)

        # plt.plot(position_x,position_y)

        # plt.show()

        return position_x,position_y

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

    #定义第一个回调函数－－轨迹追踪函数
    def callback(self,data):
        #得到理想位置和实际位置
        print("目标点")
        print(self.count)

        print("理想位置")
        print(self.x[self.count],self.y[self.count])

        print("实际位置")
        print(self.positions[0:2])

        self.x_init[0] = self.positions[0] #x
        self.x_init[1] = self.positions[1] #y
        self.x_init[2] = self.euler_angle[2] #yaw

        self.x_desire[0]=self.x[self.count]
        self.x_desire[1]=self.y[self.count]
        self.x_desire[2]=0

        # #调试代码
        # self.x_desire[0]=5
        # self.x_desire[1]=5
        # self.x_desire[2]=0

        #将理想轨迹和实际轨迹丢进MPC得出控制量
        #传入实际位置＋理想位置
        self.mpc.input_parameters(self.x_init,self.x_desire) #把实际的yaw角带进去

        #进行运动
        self.moveRobot()

        #判断是否到达目标点
        self.target_point(self.x_desire,self.positions)

    #控制两栖机器人进行运动(重要函数)
    def moveRobot(self):
        #-------直线的时候:先转到目标角度再用MPC--------
        #计算实际位置和理想点之间的向量 :目标点-当前点

        self.point_to_target[0] = self.x[self.count]-self.positions[0] #方向向量x
        self.point_to_target[1] = self.y[self.count]-self.positions[1] #方向向量y

        #求该方向向量的yaw角
        self.yaw_point_to_target = self.angle(self.point_to_target[0],self.point_to_target[1])

        #转动到期望的角度 
        #计算理想角度和实际角度之间的误差: 理想角度-实际角度
        error_angle = np.abs(self.yaw_point_to_target-self.euler_angle[2])

        if error_angle > np.pi:
            error_angle = 2*np.pi - error_angle 
        
        print("error_angle")
        print(error_angle)

        # if not error_angle < 2*np.pi/15:
        if not error_angle < np.pi/15:
            self.turn_to_target_angle(self.yaw_point_to_target)
        else:
            #得到最优控制序列
            self.control_final = self.mpc.optimize() 

            #将控制量发布出去
            self.move(self.control_final)

        # if 16<= self.count <=27 or 44<= self.count <=55: #的时候直线
        #     #转动到期望的角度 
        #     if not (self.yaw_point_to_target - np.pi/15 < self.euler_angle[2] < self.yaw_point_to_target + np.pi/15 ): #没在规定角度之内
        #         print("转动到期望的角度")
        #         self.turn_to_target_angle(self.yaw_point_to_target)
        #     else:
        #         #得到最优控制序列
        #         self.control_final = self.mpc.optimize() 

        #         #将控制量发布出去
        #         self.move(self.control_final)
        # else:
        #     #得到最优控制序列
        #     self.control_final = self.mpc.optimize() 

        #     #将控制量发布出去
        #     self.move(self.control_final)

    def target_point(self,x_desire,x_now):
        print("target to position!!!!!!!!!!!!!!!!!!!!!!!!!!")

        #计算距离
        distance = np.sqrt(np.power((x_desire[0]-x_now[0]),2)+np.power((x_desire[1]-x_now[1]),2))

        print("distance!!!!!!!!!!!!!!!!!!!")
        print(distance)

        #如果距离满足要求（目标点＋１）
        if distance <= 0.25:
            self.count = self.count+1 #目标点变成下一个

            print("达到目标点")

            #运动到终点之后，归位
            #if self.count >=7 :
            if self.count >=55 :
                self.count = 0

        #　调试：取消跳过下一个点
        # #如果走过了，进入下一个点
        # if 0<self.count<=15: #第一个正弦曲线
        #     #如果走过了（目标点+1)
        #     if x_now[0] < x_desire[0] and x_now[1] < x_desire[1]: # x_now[0]=x坐标　x_now[1]=y坐标
        #         print("走过了！！！！！！！！！！！！！！！！！！")

        #         print("x_now")
        #         print(x_now)
        #         print("x_desire")
        #         print(x_desire)

        #         self.count_number = self.count_number +1 
            
        #         if self.count_number >=2:
        #             print("跳转到下一个点！！！！！！！！！！！！！！！！！！")
        #             #跳转至下一个点
        #             self.count = self.count + 1

        #             #初始化
        #             self.count_number = 0
        # elif 16<= self.count <=27: #第一条直线
        #     #如果走过了（目标点+1)
        #     if x_now[1] > x_desire[1]: # x_now[0]=x坐标　x_now[1]=y坐标
        #         print("走过了！！！！！！！！！！！！！！！！！！")

        #         print("x_now")
        #         print(x_now)
        #         print("x_desire")
        #         print(x_desire)
                
        #         self.count_number = self.count_number +1 
            
        #         if self.count_number >=2:
        #             print("跳转到下一个点！！！！！！！！！！！！！！！！！！")
        #             #跳转至下一个点
        #             self.count = self.count + 1

        #             #初始化
        #             self.count_number = 0
        # elif 28<= self.count <=43: #第二个正弦曲线

        #     #调试：逃避第三十个点&第四十二个点
        #     if self.count == 30 or self.count == 42 :
        #         self.stepPer = 2 #一次运动步数设置为两步
        #         # self.count = self.count +1 
        #     else:
        #         self.stepPer = 1 #一次运动步数设置为两步

        #     #如果走过了（目标点+1)
        #     if x_now[1] < x_desire[1] and x_now[0] > x_desire[0]: # x_now[0]=x坐标　x_now[1]=y坐标
        #         print("走过了！！！！！！！！！！！！！！！！！！")

        #         print("x_now")
        #         print(x_now)
        #         print("x_desire")
        #         print(x_desire)
                
        #         self.count_number = self.count_number +1 
            
        #         if self.count_number >=2:
        #             print("跳转到下一个点！！！！！！！！！！！！！！！！！！")
        #             #跳转至下一个点
        #             self.count = self.count + 1

        #             #初始化
        #             self.count_number = 0

        # elif 44<= self.count <=55: #第二条直线
        #     #如果走过了（目标点+1)
        #     if x_now[1] > x_desire[1]: # x_now[0]=x坐标　x_now[1]=y坐标
        #         print("走过了！！！！！！！！！！！！！！！！！！")

        #         print("x_now")
        #         print(x_now)
        #         print("x_desire")
        #         print(x_desire)
                
        #         self.count_number = self.count_number +1 
            
        #         if self.count_number >=2:
        #             print("跳转到下一个点！！！！！！！！！！！！！！！！！！")
        #             #跳转至下一个点
        #             self.count = self.count + 1

        #             #初始化
        #             self.count_number = 0
        # else :
        #     self.count = 0

    def move(self,control_final):
        #初始化
        is_done.data = 0

        while 1:
            if(self.go_flag<self.stepPer):
                if (is_done.data==1):#静止状态
                    #连续发送五次控制量
                    for number in range(10):
                        self.data_publish(control_final[self.go_flag]) 

                    print("control_final[num]")
                    print(control_final[self.go_flag])

                    #休眠一秒钟
                    time.sleep(1)

                    #步数加1
                    self.go_flag = self.go_flag + 1 

                    #状态量复原
                    is_done_data = 0
                else : 
                    print("is_done.data == 0")
                    #状态量复原
                    is_done_data = 0

                    #暂停一秒，让机器人充分运动
                    time.sleep(1)


            elif(self.go_flag >=self.stepPer):
                #将turn_flag复位
                self.go_flag = 0

                is_done.data = 0

                #退出循环
                print("退出循环")
                
                break
            
            is_done.data=0
            time.sleep(1) #让isdone有充分时间被改变

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
        # if not error_angle_rect < 2*np.pi/15:
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


#创建主函数
if __name__ == '__main__':
    #初始化对象
    dataset = tracking_networks()

    #发布消息
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        dataset.pub.publish(state)
        rate.sleep()

    #监控pub和sub
    rospy.spin()

