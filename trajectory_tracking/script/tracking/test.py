import rospy
import numpy as np
import math 


#  #---------生成理想轨迹---------
# x=np.arange(0,-5,-1)
# y=np.arange(0,-3,-0.5)

# print(x)
# print(y)

#---------生成理想轨迹---------
x=np.arange(4,-4,-0.5)
y=np.zeros(len(x))

#插值取目标点
for num in range(len(x)):
    y[num] = 3*math.sin(x[num]*np.pi/8) # list向量


print(x)
print(y)