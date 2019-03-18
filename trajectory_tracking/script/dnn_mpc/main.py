#coding=utf-8
import rospy

#-----------personal------------
import controller

import mpc #导入相关的文件
import mpc_line 

if __name__ == '__main__':
    #init
    data = controller.control()
    
    #接收数据
    data.data_receive()
    
    

