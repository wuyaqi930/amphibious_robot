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
    # print(data)
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

    # #载入并合并txt数据
    # data_1 = np.loadtxt('C:/Users/Administrator/Desktop/data_training/dnn/data_txt/data_1.txt',dtype= 'str',skiprows=1,delimiter=",")
    # data_2 = np.loadtxt('C:/Users/Administrator/Desktop/data_training/dnn/data_txt/data_2.txt',dtype= 'str',skiprows=1,delimiter=",")
    # data_3 = np.loadtxt('C:/Users/Administrator/Desktop/data_training/dnn/data_txt/data_3.txt',dtype= 'str',skiprows=1,delimiter=",")
    # data_4 = np.loadtxt('C:/Users/Administrator/Desktop/data_training/dnn/data_txt/data_4.txt',dtype= 'str',skiprows=1,delimiter=",")
    # data_5 = np.loadtxt('C:/Users/Administrator/Desktop/data_training/dnn/data_txt/data_5.txt',dtype= 'str',skiprows=1,delimiter=",")
    # data_6 = np.loadtxt('C:/Users/Administrator/Desktop/data_training/dnn/data_txt/data_6.txt',dtype= 'str',skiprows=1,delimiter=",")
    # data_7 = np.loadtxt('C:/Users/Administrator/Desktop/data_training/dnn/data_txt/data_7.txt',dtype= 'str',skiprows=1,delimiter=",")
    # data_8 = np.loadtxt('C:/Users/Administrator/Desktop/data_training/dnn/data_txt/data_8.txt',dtype= 'str',skiprows=1,delimiter=",")
    # data_9 = np.loadtxt('C:/Users/Administrator/Desktop/data_training/dnn/data_txt/data_9.txt',dtype= 'str',skiprows=1,delimiter=",")
    # data_10 = np.loadtxt('C:/Users/Administrator/Desktop/data_training/dnn/data_txt/data_10.txt',dtype= 'str',skiprows=1,delimiter=",")
    # data_11 = np.loadtxt('C:/Users/Administrator/Desktop/data_training/dnn/data_txt/data_11.txt',dtype= 'str',skiprows=1,delimiter=",")

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
    #新更新的数据
    data_12 = np.loadtxt('/home/qi/dataset/data_txt/data_12.txt',dtype= 'str',skiprows=1,delimiter=",")
    data_13 = np.loadtxt('/home/qi/dataset/data_txt/data_13.txt',dtype= 'str',skiprows=1,delimiter=",")
    data_14 = np.loadtxt('/home/qi/dataset/data_txt/data_14.txt',dtype= 'str',skiprows=1,delimiter=",")
    data_15 = np.loadtxt('/home/qi/dataset/data_txt/data_15.txt',dtype= 'str',skiprows=1,delimiter=",")
    data_16 = np.loadtxt('/home/qi/dataset/data_txt/data_16.txt',dtype= 'str',skiprows=1,delimiter=",")
    data_17 = np.loadtxt('/home/qi/dataset/data_txt/data_17.txt',dtype= 'str',skiprows=1,delimiter=",")
    data_18 = np.loadtxt('/home/qi/dataset/data_txt/data_18.txt',dtype= 'str',skiprows=1,delimiter=",")
    data_19 = np.loadtxt('/home/qi/dataset/data_txt/data_19.txt',dtype= 'str',skiprows=1,delimiter=",")
    data_20 = np.loadtxt('/home/qi/dataset/data_txt/data_20.txt',dtype= 'str',skiprows=1,delimiter=",")
    

    #生成数据的函数
    train_data_1 = data_create(data_1,euler_angle_final) #输入全是数据
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
    #新生成的数据
    train_data_12 = data_create(data_12,euler_angle_final) #输入全是数据
    train_data_13 = data_create(data_13,euler_angle_final) #输入全是数据
    train_data_14 = data_create(data_14,euler_angle_final) #输入全是数据
    train_data_15 = data_create(data_15,euler_angle_final) #输入全是数据
    train_data_16 = data_create(data_16,euler_angle_final) #输入全是数据
    train_data_17 = data_create(data_17,euler_angle_final) #输入全是数据
    train_data_18 = data_create(data_18,euler_angle_final) #输入全是数据
    train_data_19 = data_create(data_19,euler_angle_final) #输入全是数据
    train_data_20 = data_create(data_20,euler_angle_final) #输入全是数据

    #数据合并
    data_final_1 = np.vstack((train_data_1,train_data_2,train_data_3,train_data_4,train_data_5,train_data_6,train_data_7,train_data_8,train_data_9,train_data_10,train_data_11))
    data_final_2 = np.vstack((train_data_12,train_data_13,train_data_14,train_data_15,train_data_16,train_data_17,train_data_18,train_data_19,train_data_20))

    data_final = np.vstack((data_final_1,data_final_2))

    #数据处理：dx dy 
    data_final[:,[7,8]] = data_final[:,[7,8]].astype('float64')-data_final[:,[0,1]].astype('float64') #将label前两个变成 dx dy 

    # print("行数")
    # print(data_final.shape[1])

    #处理数据：dyaw 
    #获得错误 dyaw
    yaw_before  = data_final[:,[9]].astype('float64') - data_final[:,[4]].astype('float64') 

    print(yaw_before[0:1000])

    yaw_after  = data_final[:,[9]].astype('float64') - data_final[:,[4]].astype('float64') 

    #获得正确 dyaw
    for num in range (yaw_before.shape[0]):
        if abs(yaw_before[num])>np.pi:
            if yaw_before[num]<0:
                yaw_after[num] = ((yaw_before[num])%(2*np.pi))
            else :
                yaw_after[num] = -((-yaw_before[num])%(2*np.pi))
        else: 
            yaw_after[num] = yaw_before[num]

    #将正确yaw 赋值给原来的列
    data_final[:,[9]]=yaw_after

    # print(data_final[5,[9]].dtype)
    # print(data_final[num,[9]].astype('float').dt)

    for num in range (data_final.shape[0]):
        if ((data_final[num,[9]].astype('float'))>np.pi):
            print(np.pi)
            print ("数字")
            print(num)
            print("存在无效数据")
            print(data_final[num,[9]])

    #返回最终数据
    np.save("data_final_2019_4_1.npy",data_final)


    