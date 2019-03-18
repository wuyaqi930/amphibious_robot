#coding=utf-8
#-------------导入相关安装包-------------
import numpy as np
from scipy.optimize import minimize
#import matplotlib.pyplot as plt
#import reload_dnn as reload #载入训练网络的package 
import time #导入时间
import datetime

#-------------导入ros的package-------------
import rospy
from nav_msgs. msg import Odometry #接收里程计信息
from geometry_msgs.msg import Twist #
from tf.transformations import euler_from_quaternion, quaternion_from_euler #将接收到的四元数转化为转角信息

# 定义一个模型预测控制的类 
class MPC:

    #------------1.定义数据传入函数------------
    def __init__(self,data):
    #------------初始化赋值------------
        self.data = data
    #------------初始化相关数值------------
        self.quaternion = np.zeros(4) #四元数
        self.euler_angle = np.zeros(3) #转角信息
        self.positions = np.zeros(3) #位置信息
        #self.line_finish = 19 #走一条直线花的时间

        #测试代码
        self.line_finish = -10 #走一条直线花的时间

        self.u = np.zeros(2)
        self.error = 0 #和理想轨迹的误差大小

        self.turn_flag = 0 #是否转动过
        self.straight_flag = 0 #是否直线运动过
        self.turn_flag_line = 0 #直线运动时候转角修正的flag
        self.straight_flag = 0 #没误差的时候，走到正中间去的flag(第一条直线)
        self.straight_flag_2 = 0 #没误差的时候，走到正中间去的flag(第二条直线)

        self.max_error = 0.3 #轨迹跟踪过程当中允许的最大偏离误差数值

    #------------2.开始优化------------
    def optimizer(self):
        #获取此时的position
        self.position(self.data)

        #获取此时的orientation
        self.orientation(self.data)

        #执行运动
        self.control()
        
    #------------3.相关控制函数------------
    #定义发送数据的函数:只发一次（可以考虑发很多次，最终要看效果如何）
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
    
    def position(self,data):
        #获取此时的position
        self.positions[0] = data.position.x #x坐标
        self.positions[1] = data.position.y #y坐标
        self.positions[2] = data.position.z #z坐标

        #显示此时的position
        print("x",self.positions[0])
        print("y",self.positions[1])
        print("z",self.positions[2])

    def control(self):
        #orientation的四元数转化为欧拉角
        self.euler_angle = euler_from_quaternion(self.quaternion)
        

        self.yaw = self.euler_angle[2] #获取yaw角度

        print("yaw", self.yaw*180/np.pi) #将yaw角度显示出来
        
        #对应的轨迹
        #计算实时坐标
        self.position_x = self.positions[0] #x坐标数值
        self.position_y = self.positions[1] #y坐标数值

        #----------------华丽的分割线----------------
        #----------------下面是自己写的 (2）----------------

        #自己测试代码
        if self.position_x > self.line_finish: #第一条直线(因为行走方向是负数)
            #--------调试代码--------
            print("现在还在第一条直线")

            #--------有误差时--------
            #计算和理想轨迹的误差
            # self.error = np.abs(self.position[1])
            self.error = self.positions[1]

            print("self.error")
            print(self.error)

            #矫正误差
            if self.error > self.max_error: #在右边 （感觉误差到了0.3就很多了，具体数字需要自己想办法确定下来）
                # 转到合适角度
                if (np.pi/2) < self.yaw < (np.pi) or (-np.pi) < self.yaw < (np.pi/4): # 不在理想区间内，就进入这个循环
                    self.turn(self.positions[1]) # y = self.position[1]、
                
                # 直线运动(减小误差)
                if self.turn_flag == 0: #没发生转动就直行
                    self.go_straight() #！！！：是否来得及执行

            elif self.error < -self.max_error : #在左边 （感觉误差到了0.3就很多了，具体数字需要自己想办法确定下来）
                # 转到合适角度
                if (-np.pi/4) < self.yaw < (np.pi) or (-np.pi) < self.yaw < (-np.pi/2):
                    self.turn(self.positions[1])

                #直线运动(减小误差)
                if self.turn_flag ==0: #没发生转动就直行
                    self.go_straight() #！！！：是否来得及执行

            #--------没误差时--------
            #if self.turn_flag == 0 and self.straight_flag == 0: #之前没发生直行或者转动才执行
            if -self.max_error < self.error < self.max_error :
                #--------没误差时：1.先直行一波，将误差降到 【-0.05，0.05】之间 --------
                if not -0.05< self.error<0.05 and self.straight_flag == 0:
                    self.go_straight() #！！！：是否来得及执行

                    if -0.05< self.error<0.05:
                        self.straight_flag == 1
                    print("第一条直线--没误差--缩小误差的直行")

                #--------没误差时：2.开始调整角度，前进 --------
                else:
                    #1. 转角修正
                    if not (-(np.pi)/10 < self.yaw < (np.pi)/10) :#要修正
                        self.turn_straight(self.yaw, 1) 

                    #2. 直线前行
                    if self.turn_flag_line == 0: #没发生转动就直行
                        #直线运动(减小误差)
                        self.go_straight()

            #--------flag重置--------
            # 1.有误差的fiag 
            self.turn_flag = 0 #所有程序执行完之后重置flag
            self.straight_flag = 0

            # 2.没误差的flag
            self.turn_flag_line = 0


        elif self.position_x <= self.line_finish:
            #--------调试代码--------
            print("现在还在第二条直线")

            #--------进入第二条直线--------
            if (self.line_finish-0.5) < self.position_x < self.line_finish: #先走两个直线，远离边界线
                self.go_straight() #！！！：是否来得及执行

            else:
                #--------有误差时--------
                #计算和理想轨迹的误差
                # self.error = np.abs(self.position[1])
                self.error = self.position_x - (self.line_finish-2) # self.line_finish =-10

                print("error")
                print(self.error)

                #矫正误差
                if self.error < -self.max_error: #在上边 小于-0.3 = 在距离直线更远的地方
                    #转到合适角度
                    #if (-np.pi) < self.yaw < 0 or 0 < self.yaw < (3*np.pi/4): # 这个判断有点问题
                    if (-np.pi) < self.yaw < (-np.pi*3/4) or (np.pi/4) < self.yaw < (np.pi) or (-np.pi*3/4) < self.yaw < (np.pi/4): #注意看看这个判断是不是有问题
                        self.turn_2(self.position_x) # y = self.position[1]
                    
                        print("第二条直线--上面--有误差--转弯")

                    #直线运动(减小误差)
                    if self.turn_flag ==0: #没发生转动就直行
                        self.go_straight() #！！！：是否来得及执行

                        print("第二条直线--上面--有误差--直行")

                elif self.error > self.max_error : #在下边 self.max_error = 0.3
                    #转到合适角度
                    if 0 < self.yaw <(np.pi*3/4) or (np.pi*3/4) < self.yaw < (np.pi) or (-np.pi) < self.yaw < (-np.pi/4): #注意看看这个判断是不是有问题
                        self.turn_2(self.position_x)

                        print("第二条直线--下面--有误差--转弯")

                    #直线运动(减小误差)
                    if self.turn_flag ==0: #没发生转动就直行
                        self.go_straight() #！！！：是否来得及执行
                        
                        print("第二条直线--下面--有误差--直行")

                #--------没误差时--------
                #if self.turn_flag == 0 and self.straight_flag == 0: #之前没发生直行或者转动才执行
                if -self.max_error < self.error < self.max_error :
                    #--------没误差时：1.先直行一波，将误差降到 【-0.05，0.05】之间 --------
                    if not -0.05< self.error<0.05 and self.straight_flag_2 == 0:
                        self.go_straight() #！！！：是否来得及执行

                        if -0.05< self.error<0.05:
                            self.straight_flag_2 == 1
                        print("第二条直线--没误差--缩小误差的直行")
                    
                    #--------没误差时：2.开始调整角度，前进 --------
                    else:
                        #1. 转角修正
                        if not (((-np.pi)/2-(np.pi)/10) < self.yaw < ((-np.pi)/2+(np.pi)/10)) :#要修正
                            self.turn_straight(self.yaw, 2) 

                            print("第二条直线--没误差--转弯")

                        #2. 直线前行
                        if self.turn_flag_line == 0: #没发生转动就直行
                            #直线运动(减小误差)
                            self.go_straight()

                            print("第二条直线--没误差--直行")

            #--------flag重置--------
            self.turn_flag = 0 #所有程序执行完之后重置flag
            self.straight_flag = 0

            # 2.没误差的flag
            self.turn_flag_line = 0


        #----------------华丽的分割线----------------
        #----------------原始代码 (2）----------------

        # if self.position_x < self.line_finish: #第一条直线
        #     #--------有误差时--------
        #     #计算和理想轨迹的误差
        #     # self.error = np.abs(self.position[1])
        #     self.error = self.positions[1]
        #     #矫正误差
        #     if self.error > 0.2: #在左边
        #         #转到合适角度
        #         if (-np.pi/4) < self.yaw < (np.pi) or (-np.pi) < self.yaw < (-np.pi/2): # 这个判断有点问题
        #             self.turn(self.positions[1]) # y = self.position[1]
                
        #         #直线运动(减小误差)
        #         if self.turn_flag ==0: #没发生转动就直行
        #             self.go_straight() #！！！：是否来得及执行

        #     elif self.error < -0.2 : #在右边
        #         #转到合适角度
        #         if (np.pi/2) < self.yaw < (np.pi) or (-np.pi) <self.yaw < (np.pi/4):
        #             self.turn(self.positions[1])

        #         #直线运动(减小误差)
        #         if self.turn_flag ==0: #没发生转动就直行
        #             self.go_straight() #！！！：是否来得及执行
            
        #     #--------没误差时--------
        #     #if self.turn_flag == 0 and self.straight_flag == 0: #之前没发生直行或者转动才执行
        #     if -0.2 < self.error < 0.2 :
        #         #1. 转角修正
        #         if not (-(np.pi)/10 < self.yaw < (np.pi)/10) :#要修正
        #             self.turn_straight(self.yaw, 1) 

        #         #2. 直线前行
        #         if self.turn_flag_line == 0: #没发生转动就直行
        #             #直线运动(减小误差)
        #             self.go_straight()

        #     #--------flag重置--------
        #     # 1.有误差的fiag 
        #     self.turn_flag = 0 #所有程序执行完之后重置flag
        #     self.straight_flag = 0

        #     # 2.没误差的flag
        #     self.turn_flag_line = 0

        # elif self.position_x >= self.line_finish: #第二条直线
        #     #--------有误差时--------
        #     #计算和理想轨迹的误差
        #     # self.error = np.abs(self.position[1])
        #     self.error = self.position_x - 20 #20是一条直线的长度

        #     #矫正误差
        #     if self.error > 0.2: #在上边
        #         #转到合适角度
        #         if (-np.pi) < self.yaw < 0 or 0 < self.yaw < (3*np.pi/4): # 这个判断有点问题
        #             self.turn(self.position_x) # y = self.position[1]
                
        #         #直线运动(减小误差)
        #         if self.turn_flag ==0: #没发生转动就直行
        #             self.go_straight() #！！！：是否来得及执行

        #     elif self.error < -0.2 : #在下边
        #         #转到合适角度
        #         if (-np.pi)< self.yaw <0 or (np.pi/4) < self.yaw < (np.pi):
        #             self.turn(self.position_x)

        #         #直线运动(减小误差)
        #         if self.turn_flag ==0: #没发生转动就直行
        #             self.go_straight() #！！！：是否来得及执行
            
        #     #--------没误差时--------
        #     #if self.turn_flag == 0 and self.straight_flag == 0: #之前没发生直行或者转动才执行
        #     if -0.2 < self.error < 0.2 :
        #         #1. 转角修正
        #         if not (2*(np.pi)/5 < self.yaw < 3*(np.pi)/5) :#要修正
        #             self.turn_straight(self.yaw, 2) 

        #         #2. 直线前行
        #         if self.turn_flag_line == 0: #没发生转动就直行
        #             #直线运动(减小误差)
        #             self.go_straight()

        #     #--------flag重置--------
        #     self.turn_flag = 0 #所有程序执行完之后重置flag
        #     self.straight_flag = 0
        
    
    
    def data_publish(self,u):
        #初始化publisher
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10) # topic和消息类型需要和肖岸星商量确定一下 
        
        #初始化twist
        twist = Twist()

        # 给twist赋值
        twist.linear.x = u[0] #线速度
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0 #角速度
        twist.angular.y = 0.0
        twist.angular.z = u[1]

        #调试消息
        print("twist")
        print(twist)

        #发送twist
        pub.publish(twist) #将数据发送出去

    #直线运动
    def go_straight(self):
        #定义线速度角速度
        self.u[0] = 1 #线速度为零
        self.u[1] = 0 #角速度为1，右转一圈

        #发送控制量
        self.data_publish(self.u)

        #显示控制量
        print("直行")

        #表明数据已发送
        self.straight_flag = 1

    #横向-转弯 (已经修改完成)
    def turn_2(self,position_x):

        if position_x < (self.line_finish-2) :#在上边 (< -11) self.line_finish= -10
            #判断此时的转角
            if (-np.pi*3/4) < self.yaw < (np.pi/4): #顺时针转动
                #定义线速度角速度
                self.u[0] = 0 #线速度为零
                self.u[1] = 1 #角速度为1，右转一圈

                #发送控制量
                self.data_publish(self.u)

                #显示控制量
                print("右转")

                #表明数据已发送
                self.turn_flag = 1
            elif (-np.pi) < self.yaw < (-np.pi*3/4) or (np.pi/4) < self.yaw < (np.pi): #逆时针转动
                #定义线速度角速度
                self.u[0] = 0 #线速度为零
                self.u[1] = -1 #角速度为1，左转一圈

                #发送控制量
                self.data_publish(self.u)

                #显示控制量
                print("左转")

                #表明数据已发送
                self.turn_flag = 1

        elif position_x > (self.line_finish-2) :#在下边 (>-11)
            #判断此时的转角
            if 0 < self.yaw <(np.pi*3/4): #顺时针转动
                #定义线速度角速度
                self.u[0] = 0 #线速度为零
                self.u[1] = 1 #角速度为1，右转一圈

                #发送控制量
                self.data_publish(self.u)

                #显示控制量
                print("右转")

                #表明数据已发送
                self.turn_flag = 1
            elif (np.pi*3/4) < self.yaw < (np.pi) or (-np.pi) < self.yaw < (-np.pi/4): #逆时针转动
                #定义线速度角速度
                self.u[0] = 0 #线速度为零
                self.u[1] = -1 #角速度为1，左转一圈

                #发送控制量
                self.data_publish(self.u)

                #显示控制量
                print("左转")

                #表明数据已发送
                self.turn_flag = 1

    # #横向-转弯 
    # def turn_2(self,position_x):

    #     if position_x < self.line_finish :#在上边 (< -10)
    #         #判断此时的转角
    #         if (-np.pi) < self.yaw < 0: #顺时针转动
    #             #定义线速度角速度
    #             self.u[0] = 0 #线速度为零
    #             self.u[1] = 1 #角速度为1，右转一圈

    #             #发送控制量
    #             self.data_publish(self.u)

    #             #表明数据已发送
    #             self.turn_flag = 1
    #         elif 0 < self.yaw <(3*np.pi/4): #逆时针转动
    #             #定义线速度角速度
    #             self.u[0] = 0 #线速度为零
    #             self.u[1] = -1 #角速度为1，左转一圈

    #             #发送控制量
    #             self.data_publish(self.u)

    #             #表明数据已发送
    #             self.turn_flag = 1

    #     elif position_x < 20:#在下边
    #         #判断此时的转角
    #         if (np.pi/4) < self.yaw < (np.pi): #顺时针转动
    #             #定义线速度角速度
    #             self.u[0] = 0 #线速度为零
    #             self.u[1] = 1 #角速度为1，右转一圈

    #             #发送控制量
    #             self.data_publish(self.u)

    #             #表明数据已发送
    #             self.turn_flag = 1
    #         elif (-np.pi) < self.yaw <0: #逆时针转动
    #             #定义线速度角速度
    #             self.u[0] = 0 #线速度为零
    #             self.u[1] = -1 #角速度为1，左转一圈

    #             #发送控制量
    #             self.data_publish(self.u)

    #             #表明数据已发送
    #             self.turn_flag = 1

    #纵向-转弯 （已经修改完成）
    def turn(self,position_y):

        if position_y > 0 :#在右边
            #判断此时的转角
            if (np.pi/2) < self.yaw < (np.pi): #顺时针转动
                #定义线速度角速度
                self.u[0] = 0 #线速度为零
                self.u[1] = 1 #角速度为1，右转一圈

                #发送控制量
                self.data_publish(self.u)

                #显示控制量
                print("右转")

                #表明数据已发送
                self.turn_flag = 1
            elif (-np.pi) < self.yaw < (np.pi/4): #逆时针转动
                #定义线速度角速度
                self.u[0] = 0 #线速度为零
                self.u[1] = -1 #角速度为1，左转一圈

                #发送控制量
                self.data_publish(self.u)

                #显示控制量
                print("左转")

                #表明数据已发送
                self.turn_flag = 1
        elif position_y < 0:#在左边
            #判断此时的转角
            if (-np.pi/4) < self.yaw < (np.pi): #顺时针转动
                #定义线速度角速度
                self.u[0] = 0 #线速度为零
                self.u[1] = 1 #角速度为1，右转一圈

                #发送控制量
                self.data_publish(self.u)

                #显示控制量
                print("右转")

                #表明数据已发送
                self.turn_flag = 1
            elif (-np.pi) <self.yaw < (-np.pi/2): #逆时针转动
                #定义线速度角速度
                self.u[0] = 0 #线速度为零
                self.u[1] = -1 #角速度为1，左转一圈

                #发送控制量
                self.data_publish(self.u)

                #显示控制量
                print("左转")

                #表明数据已发送
                self.turn_flag = 1

    #直线运动--转角修正（已经修改完成）
    def turn_straight(self,yaw,num):
        #第一条直线
        if num == 1 :
            if (np.pi/10) < yaw <(np.pi) : #顺势针转动 
                #定义线速度角速度
                self.u[0] = 0 #线速度为零
                self.u[1] = 1 #角速度为1，右转一圈

                #发送控制量
                self.data_publish(self.u)

                #显示控制量
                print("右转")

                #表明数据已发送
                self.turn_flag_line = 1

            if (-np.pi) < yaw <(-np.pi/10) : #逆时针转动 
                #定义线速度角速度
                self.u[0] = 0 #线速度为零
                self.u[1] = -1 #角速度为1，右转一圈

                #发送控制量
                self.data_publish(self.u)

                #显示控制量
                print("左转")

                #表明数据已发送
                self.turn_flag_line = 1

        #第二条直线
        if num == 2 :
            if ((-np.pi)/2+(np.pi)/10) < yaw < (np.pi/2) : #顺时针转动 
                #定义线速度角速度
                self.u[0] = 0 #线速度为零
                self.u[1] = 1 #角速度为1，右转一圈

                #发送控制量
                self.data_publish(self.u)

                #显示控制量
                print("右转")

                #表明数据已发送
                self.turn_flag_line = 1

            if (-np.pi) < yaw < ((-np.pi)/2-(np.pi)/10) or (np.pi/2) < yaw < (np.pi): #逆势针转动 
                #定义线速度角速度
                self.u[0] = 0 #线速度为零
                self.u[1] = -1 #角速度为1，右转一圈

                #发送控制量
                self.data_publish(self.u)

                #显示控制量
                print("左转")

                #表明数据已发送
                self.turn_flag_line = 1

























        
    # # 优化函数
    # def optimizer(self):
    #     # #调试 ：初始化代码 
    #     # rospy.init_node('mpc_line')

    #     #订阅话题
    #     rospy.Subscriber("odom", Odometry, self.callback)

    #     #始终订阅话题
    #     rospy.spin()

    #     time.sleep(1)

    # #回调函数
    # def callback(self, data): #应该是1s 1次
    #     #获取此时的position
    #     self.position(data)

    #     #获取此时的orientation
    #     self.orientation(data)

    #     #执行运动
    #     self.control()

    