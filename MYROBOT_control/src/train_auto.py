#coding=utf-8
import rospy
from std_msgs.msg import Int16
import time
from geometry_msgs.msg import Twist #导入需要的ros的package 
import numpy as np

import random #导入随机数


is_done=Int16()
is_done.data=0

#定义发送数据的函数:只发一次（可以考虑发很多次，最终要看效果如何）
def data_publish(u):
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

#发送twist：一直发指令，直到一次运动结束之后就不发指令
def callback_2(data): 
    #接受数据
    is_done.data= data.data


#定义随机控制输入
def random_key():
    #如果完成了转动才会发布指令
    if (is_done.data==1):
        #初始化控制量
        control = 0

        #生成随机数字
        rand = random.randint(0,3)

        #将数据对应成控制输入并返回
        #前进
        if rand == 0: 
            control = "w"
            print("控制命令--前进")

        #后退
        elif rand == 1:
            control = "s"
            print("控制命令--后退")

        #左转
        elif rand == 2:
            control = "a"
            print("控制命令--左转")

        #右转
        elif rand == 3:
            control = "d"
            print("右转")
        
        return control

    #如果没完成转动不发布指令
    else :
        return 0


if __name__ == '__main__':
    #初始化
    rospy.init_node('train_auto', anonymous=True)
    rospy.Subscriber("is_done", Int16, callback_2)
    start_flag = 1 #开始指令

    twist = Twist() #变量声明
    u = np.zeros(2) #控制量声明
    
    time.sleep(1)
    print(is_done.data)

    #控制说明
    print("前进：w 左拐：a 右拐:d 后退：s")
    time.sleep(0.1)

    #初始化
    control_key = 0 
    #-----------开始控制-----------

    #先发一次控制量，进行初始化（并不会运动）
    for num in range(5):
        #第一次--赋值
        u[0] = 1
        u[1] = 0
        
        #第一次--发布线速度消息
        data_publish(u)

    # time.sleep(1)

    # #第一次运动
    # for num in range(5):
    #     #第一次--赋值
    #     u[0] = 1
    #     u[1] = 0
        
    #     #第一次--发布线速度消息
    #     data_publish(u)
    
    #开始真正的控制
    while start_flag == 1: #一直循环接受数据
        #获得随机控制量
        if(is_done.data==1): #当完成动作之后才获得一个新的数值
            control_key = random_key() 
        
        #暂停一秒钟,使得control_key充分赋值
        time.sleep(1)

        if control_key == "w": #前进
            # #显示控制量
            # print("control is w")

            #没收到结束信息一直发送控制量 
            if(is_done.data==1):
                #赋值线速度
                u[0] = 1
                u[1] = 0
                
                #发布线速度消息:发送数次
                for num in range(5):
                    data_publish(u)
                
        elif control_key == "a": #左转
            # #显示控制量
            # print("control is a")

            #没收到结束信息一直发送控制量
            if(is_done.data==1):
                #赋值线速度
                u[0] = 0
                u[1] = -1
                
                #发布线速度消息
                for num in range(5):
                    data_publish(u)  

        elif control_key == "d": #右转
            # #显示控制量
            # print("control is d")

            if(is_done.data==1):
                #赋值线速度
                u[0] = 0
                u[1] = 1
                
                #发布线速度消息
                for num in range(5):
                    data_publish(u)  

        elif control_key == "s": #后退
            # #显示控制量
            # print("control is s")

            if(is_done.data==1):
                #赋值线速度
                u[0] = -1
                u[1] = 0
                
                #发布线速度消息
                for num in range(5):
                    data_publish(u)          

        
        elif control_key == "q": #六足步态
            # #显示控制量
            # print("control is q")

            if(is_done.data==1):
                #赋值线速度
                u[0] = 2
                u[1] = 0
                
                #发布线速度消息
                for num in range(5):
                    data_publish(u)

        #让机器人有时间充分反应,机器人动起来之后，便不会再发布消息
        time.sleep(1)

        #将is_down复位
        if (is_done.data == 1):

            #print("is_done已重置")
            is_done.data = 0

            #print("control_key已重置")
            control_key = 0

        if(is_done.data == 0):
            print("is_done.data = 0")

        time.sleep(0.2)


        

        