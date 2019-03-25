import numpy as np
import math
import numpy as np
import rospy #ros相关

from gazebo_msgs.msg import ModelStates #状态量数据订阅
from std_msgs.msg import Int16 #反馈需要
from sensor_msgs.msg import Imu #导入imu数据类型
from geometry_msgs.msg import Twist #控制命令需要
from std_msgs.msg import Float64MultiArray #自己的话题transformation 

import math 
import numpy as np
import time 

from tf.transformations import euler_from_quaternion, quaternion_from_euler #将接收到的四元数转化为转角信息

#全局变量赋值
is_done=Int16()
is_done.data=1

def turn_to_target_angle(euler_angle,object_angle):
        # #初始化
        # is_done.data = 0

        #调试
        print("理想角度是")
        print(object_angle*180/np.pi)

        print("实际角度是")
        print(euler_angle*180/np.pi)

        #将角度全部变为正数
        object_angle_rect=object_angle+np.pi
        euler_angle_rect=euler_angle+np.pi

        #目标角度和实际角度做差（取绝对值）
        error_angle = np.abs((object_angle_rect-euler_angle_rect))

        #如果有误差转动，没误差不转动
        if error_angle>np.pi:
            error_angle_rect = error_angle-np.pi
        else:
            error_angle_rect = error_angle

        #误差要是大于pi/15，就转动，要是小于pi/15，就不转动
        if not error_angle_rect < np.pi/15:
            if error_angle<=np.pi:
                if euler_angle_rect<object_angle_rect:#<target 逆时针
                    if(is_done.data==1):
                        #print("静止中")

                        print("逆时针旋转")
                
                    elif(is_done.data == 0):
                        print("运动中")
                if euler_angle_rect>=object_angle_rect:#>target 顺时针
                    if(is_done.data==1):
                        #print("静止中")

                        print("顺时针旋转")
                
                    elif(is_done.data == 0):
                        print("运动中")

            if error_angle>np.pi:
                if euler_angle_rect>=object_angle_rect:#>target 逆时针
                    if(is_done.data==1):
                        #print("静止中")
                        print("逆时针旋转")
                
                    elif(is_done.data == 0):
                        print("运动中")
                if euler_angle_rect<object_angle_rect:#<target 顺时针
                    if(is_done.data==1):
                        #print("静止中")
                        print("顺时针旋转")
                
                    elif(is_done.data == 0):
                        print("运动中")
        else:
            print("没有误差")

            # #已经控制完一轮了
            # self.control_once_flag = 0
        
if __name__== '__main__':
    euler_angle = np.pi/4
    target_angle = -np.pi/2

    turn_to_target_angle(euler_angle,target_angle)