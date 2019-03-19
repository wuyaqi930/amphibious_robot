import numpy as np
import math

# a_x = -1
# a_y = 0

# b_x = -2
# b_y = 2 

# cos = (a_x*b_x+a_y*b_y)/(math.sqrt(b_x*b_x+b_y*b_y)+math.sqrt(a_x*a_x+a_y*a_y))

# arccos = math.acos(cos)

# print(arccos*180/np.pi)


import numpy as np
x=np.array([-1,0])
y=np.array([-2.1,2])
# 两个向量
Lx=np.sqrt(x.dot(x))
Ly=np.sqrt(y.dot(y))
#相当于勾股定理，求得斜线的长度
cos_angle=x.dot(y)/(Lx*Ly)
#求得cos_sita的值再反过来计算，绝对长度乘以cos角度为矢量长度，初中知识。。
print(cos_angle)
angle=np.arccos(cos_angle)
angle2=angle*360/2/np.pi
#变为角度
#print(angle2)
#x.dot(y) =  y=∑(ai*bi)

if y[1]>0:
    print(-angle2)
else:
    print(angle2)
