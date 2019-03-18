import rospy #ros相关
from gazebo_msgs.msg import ModelStates #状态量数据订阅
from geometry_msgs.msg import Twist #控制命令需要 
from std_msgs.msg import Int16 #反馈需要

import numpy as np #计算科学包
from tf.transformations import euler_from_quaternion, quaternion_from_euler #将接收到的四元数转化为转角信息
import random #随机yaw角获得
from sensor_msgs.msg import Imu #导入imu数据类型
from std_msgs.msg import Float64MultiArray #data的数据类型　

import time #导入时间

#全局变量赋值
is_done=Int16()
is_done.data=0


class train_dataset():
    
    def __init__(self):
        #------函数变量------
        self.time = 0 
        self.pose_orientation = 0 
        self.horizon = 4

        #------状态量------
        self.quaternion = np.zeros(4) #四元数
        self.euler_angle = np.zeros(3) #转角信息
        self.positions = np.zeros(3) #位置信息

        self.Target_Angle_Flag=0 #期望的yaw角
        self.count =0 #计数器

        self.x_old_area=2#原始区域
        self.y_old_area=2#原始区域

        self.x_now_area = 0 #现在区域
        self.x_now_area = 0 #现在区域

        self.reigon = 3

        #------FLAG------
        self.forward_flag = 0 #直行的flag 
        self.turn_flag = 0 #转弯flag
        self.control_once_flag = 0 #控制一次的flag
        self.turn_to_target_angle_success = 0 #衡量是否转到目标角度

        #------初始化节点------
        rospy.init_node('train_data', anonymous=True) #初始化节点 

        rospy.Subscriber("imu", Imu, self.callback)  #不停触发执行循环

        rospy.Subscriber("is_done", Int16, self.callback_2) #订阅反馈

        rospy.Subscriber("gazebo/model_states", ModelStates, self.callback_3)#获得实时角度和位置

        #rospy.Subscriber("data",Float64MultiArray,self.callback_4)　#获得实时训练角度
        rospy.Subscriber("data", Float64MultiArray, self.callback_4)

        rospy.spin() #保持python活跃


    #定义第一个回调函数
    def callback(self,data):
        #降频
        if self.time % 1 == 0 :

            print(self.time)

            # #开始实现控制
            # self.control_once(np.pi/3,self.euler_angle[2]) #目标角度和此时的yaw角

            #将位姿输入控制运动：输入：euler_angle、position
            self.control(self.positions,self.euler_angle)

            # #调试代码
            # print("此时的yaw角！！！！！！！！！！！！！！！！")
            # print(self.euler_angle[2])

            self.time = self.time +1
        
        else : 
            self.time = self.time +1

    #定义第二个回调函数:实现对机器人动静状态的实时接受
    def callback_2(self,data): 
        #接受数据
        is_done.data= data.data

    #定义第三个回调函数
    def callback_3(self,data):
        #获得实际位姿
        pose_orientation = data.pose[1] #这个是实时位姿，数据类型是pose,包括position 和orientation 

        #将位姿转化并储存  
        self.position(pose_orientation) #位置
        self.orientation(pose_orientation) #姿态

    def callback_4(self,data):

        print("\n实时x!!!!!")
        print(data.data[0])
        print("实时x!!!!!")
        print(data.data[1])
        print("self.Target_Angle_Flag")
        print(self.Target_Angle_Flag)
        print("\n")

        # X所在区域
        if data.data[0]<-self.reigon :
            self.x_now_area=1

        if -self.reigon<data.data[0]<self.reigon:
            self.x_now_area=2

        if self.reigon<data.data[0] :
            self.x_now_area=3

        # Ｙ所在区域
        if data.data[1]<-self.reigon :
            self.y_now_area=1

        if -self.reigon<data.data[1]<self.reigon :
            self.y_now_area=2

        if self.reigon<data.data[1] :
            self.y_now_area=3

        print("x_now_area")
        print(self.x_now_area)
        print("y_now_area")
        print(self.y_now_area)

        print("x_old_area")
        print(self.x_old_area)
        print("y_old_area")
        print(self.y_old_area)

        if not (self.x_old_area == self.x_now_area and self.y_old_area == self.y_now_area) :
            print("机器人到了新的区域！！！！！") 

            #重新归零flag
            self.Target_Angle_Flag = 0

            #重新定义新的区域
            self.x_old_area =self.x_now_area
            self.y_old_area =self.y_now_area
        else :
            print("机器人还在旧的区域！！！！！")




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

    #关于控制的函数
    def control(self,position,orientation):

        #在区域内  注意：x = self.positions[0] y = self.positions[1]
        if -self.reigon < self.positions[0] < self.reigon and -self.reigon< self.positions[1] <self.reigon : 

            print("区域1！！！！！！")

            #先获得yaw角（只获得一次）
            if(self.Target_Angle_Flag==0):
                # self.Target_Angle = self.get_yaw(-180,180)*np.pi/180 #角度制转化成弧度制
                self.Target_Angle = self.get_yaw(-50,50)*np.pi/180 + self.euler_angle[2] #角度制转化成弧度制

                self.Target_Angle_Flag = 1

            #理想步态走一次:直行+转30圈+转到指定角度：输入（理想角度+现实角度）
            self.control_once(self.Target_Angle)

            # #此时运动已经完成判断如果不在区域之内了，就将self.turn_to_target_angle_success 和 self.Target_Angle_Flag 归零
            # if not (-4 < self.positions[0] < 4 and -4< self.positions[1] <4) :
            #     self.turn_to_target_angle_success = 0 #下一个阶段需要重新转到理想角度
            #     self.Target_Angle_Flag = 0 #下一个阶段需要重新获得理想角度
            
        #不在区域内
        else :
            #显示实时区域
            print("在区域外面")
            
            #转动１８０度
            if(self.Target_Angle_Flag==0):
                #直行一步
                print("在区域外面直行！！！！！")

                self.go_forward() #往前走
                time.sleep(2)

                #获得期望yaw角（只获得一次）
                if self.euler_angle[2]>0 :
                    self.Target_Angle = self.euler_angle[2]-np.pi #角度制转化成弧度制

                else:
                    self.Target_Angle = self.euler_angle[2]+np.pi #角度制转化成弧度制

                self.Target_Angle_Flag = 1
            
            #转弯到固定角度
            if not (self.Target_Angle - np.pi/15) < self.euler_angle[2] < (self.Target_Angle + np.pi/15):
                print("在区域外面转弯！！！！！")
                self.turn_to_target_angle(self.Target_Angle)

            else:
                #直行四步返回理想区域
                print("1 返回！！！！！")
                self.go_forward() #往前走
                time.sleep(2)

                print("2 返回！！！！！")
                self.go_forward() #往前走
                time.sleep(2)

                print("3 返回！！！！！")
                self.go_forward() #往前走
                time.sleep(2)

        #直行三步
        # elif self.positions[0] < -self.reigon and -self.reigon < self.positions[1] < self.reigon:

        #     print("区域2！！！！")

        #     #转弯函数
        #     #获得期望yaw角（只获得一次）
        #     if(self.Target_Angle_Flag==0):
        #         self.Target_Angle = self.get_yaw(140,180)*np.pi/180 #角度制转化成弧度制

        #         self.Target_Angle_Flag = 1

            
        #     #转弯到固定角度
        #     self.turn_to_target_angle(self.Target_Angle)

        #     time.sleep(1)

        #     #直行函数:转动到固定角度成功之后才执行
        #     if(self.turn_to_target_angle_success == 1):
        #         self.control_once(self.Target_Angle)

        #         # #此时运动已经完成判断如果不在区域之内了，就将self.turn_to_target_angle_success 和 self.Target_Angle_Flag 归零
        #         # if not (self.positions[0] < -4 and -4 < self.positions[1] < 4) :
        #         #     self.turn_to_target_angle_success = 0 #下一个阶段需要重新转到理想角度
        #         #     self.Target_Angle_Flag = 0 #下一个阶段需要重新获得理想角度
        
        # elif self.positions[0] >self.reigon and -self.reigon < self.positions[1] < self.reigon :

        #     print("区域3！！！！")

        #     #转弯函数
        #     #获得期望yaw角（只获得一次）
        #     if(self.Target_Angle_Flag==0):
        #         self.Target_Angle = self.get_yaw(-70,70)*np.pi/180 #角度制转化成弧度制

        #         self.Target_Angle_Flag = 1
        #     #转弯到固定角度
        #     self.turn_to_target_angle(self.Target_Angle)

        #     time.sleep(1)

        #     #直行函数:转动到固定角度成功之后才执行
        #     if(self.turn_to_target_angle_success == 1):
        #         self.control_once(self.Target_Angle)

        #         # #此时运动已经完成判断如果不在区域之内了，就将self.turn_to_target_angle_success 和 self.Target_Angle_Flag 归零
        #         # if not (self.positions[0] >4 and -4 < self.positions[1] < 4) :
        #         #     self.turn_to_target_angle_success = 0 #下一个阶段需要重新转到理想角度
        #         #     self.Target_Angle_Flag = 0 #下一个阶段需要重新获得理想角度

        # elif self.positions[1]<-self.reigon and -self.reigon < self.positions[0] <self.reigon :

        #     print("区域4！！！！")

        #     #转弯函数
        #     #获得期望yaw角（只获得一次）
        #     if(self.Target_Angle_Flag==0):
        #         #self.Target_Angle = self.get_yaw(-20,-160)*np.pi/180 #角度制转化成弧度制
        #         self.Target_Angle = self.get_yaw(-160,-20)*np.pi/180 #角度制转化成弧度制

        #         self.Target_Angle_Flag = 1
        #     #转弯到固定角度
        #     self.turn_to_target_angle(self.Target_Angle)

        #     time.sleep(1)

        #     #直行函数:转动到固定角度成功之后才执行
        #     if(self.turn_to_target_angle_success == 1):
        #         self.control_once(self.Target_Angle)

        #         # #此时运动已经完成判断如果不在区域之内了，就将self.turn_to_target_angle_success 和 self.Target_Angle_Flag 归零
        #         # if not (self.positions[1]<-4 and -4 < self.positions[0] <4) :
        #         #     self.turn_to_target_angle_success = 0 #下一个阶段需要重新转到理想角度
        #         #     self.Target_Angle_Flag = 0 #下一个阶段需要重新获得理想角度
        
        # elif self.positions[1]>self.reigon and -self.reigon < self.positions[0] <self.reigon :

        #     print("区域5！！！！")

        #     #转弯函数
        #     #获得期望yaw角（只获得一次）
        #     if(self.Target_Angle_Flag==0):
        #         self.Target_Angle = self.get_yaw(20,160)*np.pi/180 #角度制转化成弧度制

        #         self.Target_Angle_Flag = 1
        #     #转弯到固定角度
        #     self.turn_to_target_angle(self.Target_Angle)

        #     time.sleep(1)

        #     #直行函数:转动到固定角度成功之后才执行
        #     if(self.turn_to_target_angle_success == 1):
        #         self.control_once(self.Target_Angle)

        #         # #此时运动已经完成判断如果不在区域之内了，就将self.turn_to_target_angle_success 和 self.Target_Angle_Flag 归零
        #         # if not (self.positions[1]>4 and -4<self.positions[0]<4) :
        #         #     self.turn_to_target_angle_success = 0 #下一个阶段需要重新转到理想角度
        #         #     self.Target_Angle_Flag = 0 #下一个阶段需要重新获得理想角度

        # elif self.positions[0]<-self.reigon and self.positions[1]<-self.reigon :
        #     print("区域6！！！！")

        #     #转弯函数
        #     #获得期望yaw角（只获得一次）
        #     if(self.Target_Angle_Flag==0):
        #         #self.Target_Angle = self.get_yaw(-110,-160)*np.pi/180 #角度制转化成弧度制
        #         self.Target_Angle = self.get_yaw(-160,-110)*np.pi/180 #角度制转化成弧度制

        #         self.Target_Angle_Flag = 1
        #     #转弯到固定角度
        #     self.turn_to_target_angle(self.Target_Angle)

        #     time.sleep(1)

        #     #直行函数:转动到固定角度成功之后才执行
        #     if(self.turn_to_target_angle_success == 1):
        #         self.control_once(self.Target_Angle)

        #         # #此时运动已经完成判断如果不在区域之内了，就将self.turn_to_target_angle_success 和 self.Target_Angle_Flag 归零
        #         # if not (self.positions[0]<-4 and self.positions[1]<-4) :
        #         #     self.turn_to_target_angle_success = 0 #下一个阶段需要重新转到理想角度
        #         #     self.Target_Angle_Flag = 0 #下一个阶段需要重新获得理想角度
        
        # elif self.positions[0]>self.reigon and self.positions[1]<-self.reigon :
        #     print("区域7！！！！")

        #     #转弯函数
        #     #获得期望yaw角（只获得一次）
        #     if(self.Target_Angle_Flag==0):
        #         #self.Target_Angle = self.get_yaw(-20,-70)*np.pi/180 #角度制转化成弧度制
        #         self.Target_Angle = self.get_yaw(-70,-20)*np.pi/180 #角度制转化成弧度制

        #         self.Target_Angle_Flag = 1
        #     #转弯到固定角度
        #     self.turn_to_target_angle(self.Target_Angle)

        #     time.sleep(1)

        #     #直行函数:转动到固定角度成功之后才执行
        #     if(self.turn_to_target_angle_success == 1):
        #         self.control_once(self.Target_Angle)

        #         # #此时运动已经完成判断如果不在区域之内了，就将self.turn_to_target_angle_success 和 self.Target_Angle_Flag 归零
        #         # if not (self.positions[1]<-4 and self.positions[0]>4) :
        #         #     self.turn_to_target_angle_success = 0 #下一个阶段需要重新转到理想角度
        #         #     self.Target_Angle_Flag = 0 #下一个阶段需要重新获得理想角度
        
        # elif self.positions[1]>self.reigon and self.positions[0]>self.reigon :

        #     print("区域8！！！！")

        #     #转弯函数
        #     #获得期望yaw角（只获得一次）
        #     if(self.Target_Angle_Flag==0):
        #         self.Target_Angle = self.get_yaw(20,70)*np.pi/180 #角度制转化成弧度制

        #         self.Target_Angle_Flag = 1
        #     #转弯到固定角度
        #     self.turn_to_target_angle(self.Target_Angle)

        #     time.sleep(1)

        #     #直行函数:转动到固定角度成功之后才执行
        #     if(self.turn_to_target_angle_success == 1):
        #         self.control_once(self.Target_Angle)

        #         # #此时运动已经完成判断如果不在区域之内了，就将self.turn_to_target_angle_success 和 self.Target_Angle_Flag 归零
        #         # if not (self.positions[1]>4 and self.positions[0]>4) :
        #         #     self.turn_to_target_angle_success = 0 #下一个阶段需要重新转到理想角度
        #         #     self.Target_Angle_Flag = 0 #下一个阶段需要重新获得理想角度
        
        # elif self.positions[1]>self.reigon and self.positions[0]<-self.reigon :

        #     print("区域9！！！！")

        #     #转弯函数
        #     #获得期望yaw角（只获得一次）
        #     if(self.Target_Angle_Flag==0):
        #         self.Target_Angle = self.get_yaw(110,160)*np.pi/180 #角度制转化成弧度制

        #         self.Target_Angle_Flag = 1
        #     #转弯到固定角度
        #     self.turn_to_target_angle(self.Target_Angle)

        #     time.sleep(1)

        #     #直行函数:转动到固定角度成功之后才执行
        #     if(self.turn_to_target_angle_success == 1):
        #         self.control_once(self.Target_Angle)

        #         # #此时运动已经完成判断如果不在区域之内了，就将self.turn_to_target_angle_success 和 self.Target_Angle_Flag 归零
        #         # if not (self.positions[1]>4 and self.positions[0]<-4) :
        #         #     self.turn_to_target_angle_success = 0 #下一个阶段需要重新转到理想角度
        #         #     self.Target_Angle_Flag = 0 #下一个阶段需要重新获得理想角度



    #控制运动一次：直行+转30圈+转圈到指定位置
    def control_once(self,target_angle): #必须很快速 
        #阶段2
        if not self.count == 0 :
            print("阶段2")

            #调试 
            print("当前yaw角")
            print(self.euler_angle[2])

            print("理想yaw角")
            print(target_angle)

            self.turn_to_target_angle(target_angle)

            # #归零 （不能在这里归零，不然运动中，还会执行，这样就没办法进入转动状态了
            # self.count = 0 #调试代码

        #阶段1
        if self.count == 0 :
            print("阶段1")
            self.forward_turn_30()

            # time.sleep(1)
            self.count = self.count +1 

    #直行函数:直行1步+转圈30次左右
    def forward_turn_30 (self):
        #------初始化------
        self.go_forward() #往前走

        time.sleep(2) #充分运动

        #------直行------
        self.go_forward() #往前走

        time.sleep(2) #充分运动

        #------直行------
        self.go_forward() #往前走

        time.sleep(2) #充分运动

        #------转圈------
        self.turn_30() #转30圈

        # time.sleep(2) #充分运动

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

    #运动---转动30圈       
    def turn_30(self):
        #初始化
        is_done.data = 0

        # print("is_done")
        # print(is_done.data)

        while(1):
            #静止时候赋值运动：转弯
            if(self.turn_flag < 2):
                if(is_done.data==1):
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

                    #调试
                    print("转圈-30")
                    print("self.turn_flag")
                    print(self.turn_flag)

                    #转弯圈数加1
                    self.turn_flag = self.turn_flag + 1 

                    #将is_down复位
                    # print("is_done已重置")
                    is_done.data = 0
                else : 
                    print("is_done.data == 0")

                    #暂停一秒，让机器人充分运动
                    time.sleep(1)

            #转弯了30次之后退出循环
            elif(self.turn_flag >= 2):
                #将turn_flag复位
                self.turn_flag = 0

                is_done.data = 0

                #退出循环
                print("退出循环")
                
                break

    #运动---转动至期望角度
    def turn_to_target_angle(self,object_angle):
        # #初始化
        # is_done.data = 0

        #调试
        print("理想角度是")
        print(object_angle)

        print("实际角度是")
        print(self.euler_angle[2])
        
        #执行控制 self.euler_angle[2] = yaw (实时的)
        if not (object_angle - np.pi/15) < self.euler_angle[2] < (object_angle + np.pi/15): 
            if (self.euler_angle[2]<object_angle): #逆时针转动
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

            elif(self.euler_angle[2]>=object_angle): #顺时针转动
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
            print("没有误差")

            # #已经控制完一轮了
            # self.control_once_flag = 0

            #模式切换成功归零
            self.count = 0
            
            #表明转换到目标角度之间成功
            self.turn_to_target_angle_success = 1


#-------------一些封装好的函数--------------
    #获得理想yaw角的函数 
    def get_yaw(self,min_angle,max_angle):
        target_angle = random.randint(min_angle,max_angle) #!!!随后可能要置零

        return target_angle

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

    dataset = train_dataset()
