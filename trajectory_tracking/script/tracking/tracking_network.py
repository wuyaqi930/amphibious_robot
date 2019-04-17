#coding=utf-8
#------------------导入官方package------------------
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

        #初始化mpc
        self.mpc = optimize.MPC(self.x_init,self.x_desire) #初始化
        self.mpc.restore_params() #重新载入网络

        #---------生成理想轨迹---------
        #self.x=np.arange(-4,4,0.5)
        self.x=np.arange(4,-4,-0.5)
        self.y=np.zeros(len(self.x))

        #插值取目标点
        for num in range(len(self.x)):
            self.y[num] = 3*math.sin(self.x[num]*np.pi/8) # list向量

        self.count = 0 #第几个目标点

        print(self.x)
        print(self.y)

        # #---------生成理想轨迹---------
        # self.x=np.arange(0,-5,-1)
        # self.y=np.arange(0,-3,-0.5)

        #---------跟踪算法-----------
        #跟踪
        rospy.init_node("tracking_networks",anonymous=True) #初始化节点

        #发布位姿信息
        self.pub = rospy.Publisher("transformation",Float64MultiArray,queue_size=3)

        #订阅位姿信息
        rospy.Subscriber("imu", Imu, self.callback)  #不停触发执行实时轨迹跟踪主函数

        rospy.Subscriber("is_done", Int16, self.callback_2) #订阅反馈：静止发送is_done=1，运动不发送任何消息

        rospy.Subscriber("gazebo/model_states", ModelStates, self.callback_3)#获得实时角度和位置


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
        # self.x_desire[0]=4.5
        # self.x_desire[1]=3.5
        # self.x_desire[2]=0

        #将理想轨迹和实际轨迹丢进MPC得出控制量
        #传入实际位置＋理想位置
        self.mpc.input_parameters(self.x_init,self.x_desire) #把实际的yaw角带进去

        #得到最优控制序列
        self.control_final = self.mpc.optimize() 

        #将控制量发布出去
        self.move(self.control_final)

        #判断是否到达目标点
        self.target_point(self.x_desire,self.positions)

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
            if self.count >=15 :
                self.count = 0
        
        # #如果走过了（目标点+1)
        # #if x_now[0]>x_desire[0] or x_now[1]<x_desire[1]: # x_now[0]=x坐标　x_now[1]=y坐标
        
        # if x_now[0] < x_desire[0] or x_now[1] < x_desire[1]: # x_now[0]=x坐标　x_now[1]=y坐标
        #     print("走过了！！！！！！！！！！！！！！！！！！")

        #     print("x_now")
        #     print(x_now)
        #     print("x_desire")
        #     print(x_desire)

        #     self.count_number = self.count_number +1 
            
        #     if self.count_number >=1:
        #         print("跳转到下一个点！！！！！！！！！！！！！！！！！！")
        #         #跳转至下一个点
        #         self.count = self.count + 1

        #         #运动到终点之后，归位
        #         #if self.count >=7 :
        #         if self.count >=15 :
        #             self.count = 0

        #         #初始化
        #         self.count_number = 0

                

    def move(self,control_final):
        #初始化
        is_done.data = 0

        while 1:
            if(self.go_flag<2):
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


            elif(self.go_flag >= 2):
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

