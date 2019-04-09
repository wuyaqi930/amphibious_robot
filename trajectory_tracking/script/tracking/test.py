import rospy
import numpy as np
# # ------------计算实际角度和理想角度之间的距离------------
# def yawToTarget():
#     #计算实际位置和理想点之间的向量 :目标点-当前点
#     self.point_to_target[0] = self.x[self.count]-self.positions[0] #方向向量x
#     self.point_to_target[1] = self.y[self.count]-self.positions[1] #方向向量y

#     #计算理想yaw角
#     self.yaw_point_to_target = self.angle(self.point_to_target[0],self.point_to_target[1])

#     #计算实际yaw角
#     self.yaw_current = self.yaw 

#     #计算实际yaw角和理想yaw角之间的距离
#     error_yaw = self.yaw_point_to_target - self.yaw 

#     return error_yaw


#求实时和期望的夹角
def angle(point_x,point_y):
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

result = angle(-1,1)

print(result*180/np.pi)

print(np.power(result*180/np.pi,2))


# # yaw角误差
# error_yaw = self.yawToTarget()

# # 定义代价函数(不考虑yaw角，只考虑ｘ和y)
# J = self.Q*np.dot((x_now[0:2]-x_desire[0:2]),(x_now[0:2]-x_desire[0:2]).T) +self.P*np.power(error_yaw,2)# 二次型的样子