from tf.transformations import euler_from_quaternion, quaternion_from_euler #将接收到的四元数转化为转角信息
import numpy as np 

c = np.load("data_final.npy")

print(c.shape)
