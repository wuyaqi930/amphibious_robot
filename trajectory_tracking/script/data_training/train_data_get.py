import numpy as np 
from tf.transformations import euler_from_quaternion, quaternion_from_euler #将接收到的四元数转化为转角信息

#-------------得到训练数据---------------


#生成训练数据
def data_create(data,euler_angle_final):
    #获取x,y
    data_x_y = data[0:-1,2:4]
    # print(data_x_y)

    #四元素转换成欧拉角
    #data_quaternion=data[0:-1,4:8]
    data_quaternion=data[:,4:8]
    
    # print("data_quaternion")
    # print(data_quaternion)

    for num in range(data_quaternion.shape[0]) :# data_quaternion.shape[0] = 数据行数
        euler_angle = euler_from_quaternion(data_quaternion[num,:]) # num具体行数 
        
        euler_angle_final = np.vstack((euler_angle_final,euler_angle))

    # print("euler_angle_final")
    # print(euler_angle_final)

    euler_angle = euler_angle_final[1:-1,:] #第一行是[0,0,0],可以删除
    
    # print("euler_angle_final")
    # print(euler_angle_final)

    # print(data[1:,2:4].shape)
    # print(euler_angle_final[:,:].shape)

    #获取对应的x+1,y+1 
    x_y_plus_one = data[1:,2:4]
    yaw = euler_angle_final[2:,2].reshape(((data.shape[0]-1),1)) #必须要有reshape 不然会被自动变成一个向量

    label = np.hstack((x_y_plus_one,yaw)) #包含x y yaw

    #获取控制量 
    data_control = data[0:-1,8:10]

    #将数据按照列合并起来 data = data_x_y + euler_angle_final + data_label
    # print(data_x_y.shape)
    # print(euler_angle_final.shape)
    # print(data_label.shape)
    # print(data_control.shape)

    data = np.hstack((data_x_y,euler_angle,data_control,label))

    #返回训练数据
    print(data)
    return data 



if __name__ == '__main__':
    # #初始化全局
    # # euler_angle_final = np.zeros(3) #处理完之后的欧拉角,初始化为零
    # euler_angle_final = np.zeros(3) #处理完之后的欧拉角,初始化为零

    # #载入并合并txt数据
    # data_1 = np.loadtxt('/home/qi/dataset/data_txt/data_1.txt',dtype= 'str',skiprows=1,delimiter=",")
    # # data_2 = np.loadtxt('/home/qi/dataset/data_txt/data_2.txt',dtype= 'str',skiprows=1,delimiter=",")
    # # data_3 = np.loadtxt('/home/qi/dataset/data_txt/data_3.txt',dtype= 'str',skiprows=1,delimiter=",")
    # # data_4 = np.loadtxt('/home/qi/dataset/data_txt/data_4.txt',dtype= 'str',skiprows=1,delimiter=",")
    # # data_5 = np.loadtxt('/home/qi/dataset/data_txt/data_5.txt',dtype= 'str',skiprows=1,delimiter=",")
    # # data_6 = np.loadtxt('/home/qi/dataset/data_txt/data_6.txt',dtype= 'str',skiprows=1,delimiter=",")
    # # data_7 = np.loadtxt('/home/qi/dataset/data_txt/data_7.txt',dtype= 'str',skiprows=1,delimiter=",")
    # # data_8 = np.loadtxt('/home/qi/dataset/data_txt/data_8.txt',dtype= 'str',skiprows=1,delimiter=",")
    # # data_9 = np.loadtxt('/home/qi/dataset/data_txt/data_9.txt',dtype= 'str',skiprows=1,delimiter=",")
    # # data_10 = np.loadtxt('/home/qi/dataset/data_txt/data_10.txt',dtype= 'str',skiprows=1,delimiter=",")
    # # data_11 = np.loadtxt('/home/qi/dataset/data_txt/data_11.txt',dtype= 'str',skiprows=1,delimiter=",")


    # #生成数据的函数
    # train_data_1 = data_create(data_1[0:5,:],euler_angle_final) #输入全是数据
    # # train_data_2 = data_create(data_2,euler_angle_final) #输入全是数据
    # # train_data_3 = data_create(data_3,euler_angle_final) #输入全是数据
    # # train_data_4 = data_create(data_4,euler_angle_final) #输入全是数据
    # # train_data_5 = data_create(data_5,euler_angle_final) #输入全是数据
    # # train_data_6 = data_create(data_6,euler_angle_final) #输入全是数据
    # # train_data_7 = data_create(data_7,euler_angle_final) #输入全是数据
    # # train_data_8 = data_create(data_8,euler_angle_final) #输入全是数据
    # # train_data_9 = data_create(data_9,euler_angle_final) #输入全是数据
    # # train_data_10 = data_create(data_10,euler_angle_final) #输入全是数据
    # # train_data_11 = data_create(data_11,euler_angle_final) #输入全是数据
    
    # # #数据合并
    # # data_final = np.vstack((train_data_1,train_data_2,train_data_3,train_data_4,train_data_5,train_data_6,train_data_7,train_data_8,train_data_9,train_data_10,train_data_11))

    # #数据保存
    # #np.save("data_final.npy",data_final)
    # np.save("train_data_1.npy",train_data_1)

    # print(data_1[0:5,:])



    #初始化全局
    # euler_angle_final = np.zeros(3) #处理完之后的欧拉角,初始化为零
    euler_angle_final = np.zeros(3) #处理完之后的欧拉角,初始化为零

    #载入并合并txt数据
    data_1 = np.loadtxt('/home/qi/dataset/data_txt/data_1.txt',dtype= 'str',skiprows=1,delimiter=",")
    data_2 = np.loadtxt('/home/qi/dataset/data_txt/data_2.txt',dtype= 'str',skiprows=1,delimiter=",")
    data_3 = np.loadtxt('/home/qi/dataset/data_txt/data_3.txt',dtype= 'str',skiprows=1,delimiter=",")
    data_4 = np.loadtxt('/home/qi/dataset/data_txt/data_4.txt',dtype= 'str',skiprows=1,delimiter=",")
    data_5 = np.loadtxt('/home/qi/dataset/data_txt/data_5.txt',dtype= 'str',skiprows=1,delimiter=",")
    data_6 = np.loadtxt('/home/qi/dataset/data_txt/data_6.txt',dtype= 'str',skiprows=1,delimiter=",")
    data_7 = np.loadtxt('/home/qi/dataset/data_txt/data_7.txt',dtype= 'str',skiprows=1,delimiter=",")
    data_8 = np.loadtxt('/home/qi/dataset/data_txt/data_8.txt',dtype= 'str',skiprows=1,delimiter=",")
    data_9 = np.loadtxt('/home/qi/dataset/data_txt/data_9.txt',dtype= 'str',skiprows=1,delimiter=",")
    data_10 = np.loadtxt('/home/qi/dataset/data_txt/data_10.txt',dtype= 'str',skiprows=1,delimiter=",")
    data_11 = np.loadtxt('/home/qi/dataset/data_txt/data_11.txt',dtype= 'str',skiprows=1,delimiter=",")


    #生成数据的函数
    train_data_1 = data_create(data_1[0:5,:],euler_angle_final) #输入全是数据
    train_data_2 = data_create(data_2,euler_angle_final) #输入全是数据
    train_data_3 = data_create(data_3,euler_angle_final) #输入全是数据
    train_data_4 = data_create(data_4,euler_angle_final) #输入全是数据
    train_data_5 = data_create(data_5,euler_angle_final) #输入全是数据
    train_data_6 = data_create(data_6,euler_angle_final) #输入全是数据
    train_data_7 = data_create(data_7,euler_angle_final) #输入全是数据
    train_data_8 = data_create(data_8,euler_angle_final) #输入全是数据
    train_data_9 = data_create(data_9,euler_angle_final) #输入全是数据
    train_data_10 = data_create(data_10,euler_angle_final) #输入全是数据
    train_data_11 = data_create(data_11,euler_angle_final) #输入全是数据
    
    #数据合并
    data_final = np.vstack((train_data_1,train_data_2,train_data_3,train_data_4,train_data_5,train_data_6,train_data_7,train_data_8,train_data_9,train_data_10,train_data_11))

    #数据保存
    np.save("data_final.npy",data_final)


    