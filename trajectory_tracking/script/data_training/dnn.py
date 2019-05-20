import torch 
import torch.nn.functional as F
import matplotlib.pyplot as plt
import numpy as np
np.set_printoptions(threshold=np.inf)

import torch.utils.data as Data
from sklearn.model_selection import train_test_split
device = 'cuda' if torch.cuda.is_available() else 'cpu'

class neural_network:

    #------------1.初始化------------
    def __init__(self,data,BATCH_SIZE): 

        #初始化赋值
        self.BATCH_SIZE = BATCH_SIZE #训练批处理size
        self.epoch = 100 #训练层数

        #------------1.1 数据读取------------
        #生成输入数据
        self.data_input_numpy = data[:,[0,1,4,5,6]].astype(np.float)
        self.data_output_numpy = data[:,7:10].astype(np.float)
        


        #调试代码
        #print("self.data_input_numpy")
        #print(self.data_input_numpy[0:10,:])

        #print("self.data_output_numpy")
        #print(self.data_output_numpy[0:10,:])

        #self.data_input_numpy[:,[3,4]] = self.data_input_numpy[:,[3,4]]*10 #将yaw放大十倍
        

        # ------------1.2 产生训练集和测试集----------

        data_train, data_test, label_train, label_test = self.data_loader()

        #得到训练集和测试集的数据量  
        self.train_data_number = data_train.shape[0] # 0表示行 = 训练集数据量
        self.test_data_number = data_test.shape[0]  # 0表示列 = 测试集数据量

        print("训练集数据量")
        print(self.train_data_number)

        print("测试集数据量")
        print(self.test_data_number)

        # ------------1.3 将numpy转化成tensor----------

        #训练集
        data_train_torch = torch.from_numpy(data_train)
        self.data_train_torch = data_train_torch.float() # 转化成浮点数(重点语句不要错过）

        data_label_torch = torch.from_numpy(label_train)
        self.data_label_torch = data_label_torch.float() # 转化成浮点数(重点语句不要错过）

        #print(self.data_label_torch[0:1000,[2]])

        #测试集
        data_test_torch = torch.from_numpy(data_test)
        self.data_test_torch = data_test_torch.float() # 转化成浮点数(重点语句不要错过）

        label_test_torch = torch.from_numpy(label_test)
        self.label_test_torch = label_test_torch.float() # 转化成浮点数(重点语句不要错过）

        print("初始化成功")

    #----------------2.构建神经网络----------------
    def dnn(self):

        # ------------2.1进行批训练-----------

        # ------------2.1.1载入训练集-----------

        # 定义数据库 （输入输出分别是之前的输入输出）
        dataset_train = Data.TensorDataset(self.data_train_torch, self.data_label_torch) 

        # 定义数据加载器
        loader_train = Data.DataLoader(dataset = dataset_train, batch_size = self.BATCH_SIZE, shuffle = True, num_workers = 2)

        # ------------2.1.2载入测试集-----------

        # 定义数据库 （输入输出分别是之前的输入输出）-----一次性输入所有测试数据，计算loss总和 
        dataset_test = Data.TensorDataset(self.data_test_torch, self.label_test_torch) 

        # 定义数据加载器
        #loader_test = Data.DataLoader(dataset = dataset_test, batch_size = self.BATCH_SIZE, shuffle = True, num_workers = 2)
        loader_test = Data.DataLoader(dataset = dataset_test, batch_size = self.test_data_number, shuffle = True, num_workers = 2) #一次性把所有数据载入进行批训练


        #------------2.2初始化部分数据-----------
        #定义迭代次数 
        #times = self.data_size

        self.one_train_times = int((self.train_data_number/self.BATCH_SIZE)) #训练集一次训练的次数
        self.train_times = int(self.epoch*(self.train_data_number/self.BATCH_SIZE)) #训练集：批训练总次数 *层数
        self.test_times = int(self.epoch) # 测试集显示次数  = 训练层数

        #print("train_times")
        #print(self.train_times)

        #print("one_train_times")
        #print(self.one_train_times)

        #print("self.test_times")
        #print(self.test_times)

        # 绘图：生成随机输出变量（训练集）
        self.error_train = torch.zeros(self.train_times,3) 
        self.loss_train = torch.zeros(self.train_times,1) 
        

        # 绘图：生成损失函数误差变量（测试集）
        self.error_test = torch.zeros(self.test_times,1) 
        self.loss_test = torch.zeros(self.test_times,1) 

        #----------------2.3构建网络-----------------
        class Net(torch.nn.Module):
            def __init__(self, n_feature, n_hidden, n_output):
                super(Net,self).__init__()

                #不做batch_normalization
                self.hidden = torch.nn.Sequential(
                    torch.nn.Linear(n_feature, n_hidden),
                    torch.nn.LeakyReLU(),
                    torch.nn.Linear(n_hidden, n_hidden),
                    torch.nn.LeakyReLU(),
                    torch.nn.Linear(n_hidden, n_hidden),
                    torch.nn.LeakyReLU(),
                    torch.nn.Linear(n_hidden, n_hidden),
                    torch.nn.LeakyReLU()
                )

                ##做batch_normalization
                #self.hidden = torch.nn.Sequential(
                #    #torch.nn.BatchNorm1d(n_feature),
                #    torch.nn.Linear(n_feature, n_hidden),
                #    torch.nn.LeakyReLU(negative_slope=0.02),
                #    torch.nn.Linear(n_hidden, n_hidden),
                #    torch.nn.LeakyReLU(negative_slope=0.02),
                #    torch.nn.Linear(n_hidden, n_hidden),
                #    torch.nn.LeakyReLU(negative_slope=0.02),
                #    torch.nn.Linear(n_hidden, n_hidden),
                #    torch.nn.LeakyReLU(negative_slope=0.02)
                #)

                self.out = torch.nn.Linear(n_hidden, n_output)

            def forward(self, x):
                x = self.hidden(x)
                out = self.out(x)
                return out

        net = Net(n_feature=5, n_hidden=64, n_output=3)
        net.load_state_dict(torch.load('xiao.pkl'))
        print(net)

        #----------------2.4定义优化方法&定义损失函数-----------------

        ##使用“随机梯度下降法”进行参数优化
        optimizer = torch.optim.SGD(net.parameters(), lr=0.01)  # 传入 net 的所有参数, 学习率
        #
        #使用“ADAM”进行参数优化
        # optimizer = torch.optim.Adam(net.parameters(), lr=0.00005) # 传入 net 的所有参数, 学习率
        # optimizer = torch.optim.Adam(net.parameters(), lr=0.00005)  # 传入 net 的所有参数, 学习率

        #定义损失函数，计算均方差
        loss_func = torch.nn.MSELoss()      # 预测值和真实值的误差计算公式 (均方差)
        #loss_func = torch.nn.L1Loss()

        #----------------2.5使用cuda进行GPU计算-----------------
        net.to(device)
        loss_func.to(device)

        #----------------2.6具体训练过程-----------------
        
        # 训练集
        for epoch in range(self.epoch):
            #训练网络
            for step, (batch_x, batch_y) in enumerate(loader_train):
                
                #产生prediction
                prediction = net( batch_x.to(device) )     # input x and predict based on x

                #计算loss
                loss = loss_func(prediction, batch_y.to(device))     # must be (1. nn output, 2. target)

                print ("训练集loss")
                print (loss)

                #将误差函数画出来
                self.loss_train[int((epoch*self.one_train_times)+step),:] = loss #将每一个批次都记录一下

                #调试代码
                #print("epoch")
                #print(epoch)

                #print("step")
                #print(step)

                #print("epoch*step")
                #print(int((epoch*self.one_train_times)+step))

                #计算optimize
                optimizer.zero_grad()   # clear gradients for next train
                loss.backward()         # backpropagation, compute gradients
                optimizer.step()        # apply gradients

                #计算误差绝对值,并储存在data_plot当中
                percent = prediction - batch_y.to(device) #直接计算误差大小

                print("训练集error")
                # print(torch.cat((percent, batch_x), 1))
                print(percent[2])

                self.error_train[int((epoch*self.one_train_times)+step),:] = percent[0,:]  #将每一个批次的一个误差记录一下
                torch.save(net.state_dict(), 'xiao.pkl')

                #将测试集loss画出来(一次性载入所有数据）
            for step,(batch_x, batch_y) in enumerate(loader_test):
                #产生prediction
                prediction = net( batch_x.to(device) )

                #计算总误差
                error_test = prediction - batch_y.to(device) #计算绝对值误差 4157*3

                #取总误差绝对值
                error_test_abs = torch.abs(error_test)

                #总误差绝对值行列相加 
                error_sum = torch.sum(error_test_abs)

                print("error_sum")
                print(error_sum )

                #赋值给error_plot 
                self.error_test[epoch,:]= error_sum 

                #计算总loss
                loss = loss_func(prediction, batch_y.to(device))

                print("测试集loss")
                print(loss)

                #赋值给loss_plot 
                self.loss_test[epoch,:]=loss


        #将训练完的网络保存
        torch.save(net.state_dict(), 'xiao.pkl')   

    #----------------3.将函数loss画出来----------------
    def plot_loss(self):

        #将数据从tensor转化成numpy
        loss_train_numpy = self.loss_train.detach().numpy() #self 
        loss_test_numpy = self.loss_test.detach().numpy() #self 

        #将误差平均值可视化
        X = np.linspace(1, self.train_times, self.train_times, endpoint=True) #self
        X_test = np.linspace(1, self.test_times, self.test_times, endpoint=True) #self

        #绘图:两个子图 
        plt.figure(1)

        plt.subplot(211)
        plt.xlabel('Time(s)')
        plt.ylabel("loss of train")
        plt.xlim(0, self.train_times) #可能需要人为调整
        plt.plot(X,loss_train_numpy)

        plt.subplot(212)
        plt.xlabel('Time(s)')
        plt.ylabel("loss of test")
        plt.xlim(0, self.test_times) #可能需要人为调整
        plt.plot(X_test,loss_test_numpy)

        plt.show()

    #----------------3.将函数loss画出来----------------
    def plot_error(self):

        #----------------3.1训练集&测试集----------------
        #将数据从tensor转化成numpy
        error_train_numpy = self.error_train.detach().numpy() #self 
        #取每个元素的绝对值
        error_train_numpy_abs = np.abs(error_train_numpy.T) #需要进行转置才能得到相关数据
        #调用sum函数，将每列数据加起来，求误差的平均数
        average_train = np.sum(error_train_numpy_abs, axis=0)/3 #要除3，表示加起来求平均

        #将数据从tensor转化成numpy
        error_test_numpy = self.error_test.detach().numpy() #self 

        ##取每个元素的绝对值
        #error_test_numpy_abs = np.abs(error_test_numpy.T) #需要进行转置才能得到相关数据

        ##调用sum函数，将每列数据加起来，求误差的平均数
        #average_test = np.sum(error_test_numpy_abs, axis=0)/3 #要除3，表示加起来求平均

        #重新resize 
        average_test = np.reshape(error_test_numpy,(self.test_times,))

        #----------------3.2 画图----------------
        #将误差平均值可视化
        X = np.linspace(1, self.train_times, self.train_times, endpoint=True) #self
        X_test = np.linspace(1, self.test_times, self.test_times, endpoint=True) #self

        #绘图
        plt.figure(1)

        plt.subplot(211)
        plt.xlabel('Time(s)')
        plt.ylabel("error of train")
        plt.xlim(0, self.train_times) #可能需要人为调整
        plt.plot(X,average_train)

        plt.subplot(212)
        plt.xlabel('Time(s)')
        plt.ylabel("error of test")
        plt.xlim(0, self.test_times) #可能需要人为调整
        plt.plot(X_test,average_test)
        
        plt.show()
    
    def data_loader(self):
        #产生训练集和测试集
        data_train, data_test, label_train, label_test = train_test_split(self.data_input_numpy, self.data_output_numpy, test_size=0.1, random_state=42)

        #将训练集和测试集返回
        return data_train, data_test, label_train, label_test

    def restore_params(self):
        #定义网络类型
        class Net(torch.nn.Module):
            def __init__(self, n_feature, n_hidden, n_output):
                super(Net,self).__init__()
                self.hidden = torch.nn.Sequential(
                    torch.nn.Linear(n_feature, n_hidden),
                    torch.nn.LeakyReLU(),
                    torch.nn.Linear(n_hidden, n_hidden),
                    torch.nn.LeakyReLU(),
                    torch.nn.Linear(n_hidden, n_hidden),
                    torch.nn.LeakyReLU(),
                    torch.nn.Linear(n_hidden, n_hidden),
                    torch.nn.LeakyReLU(),
                )
                self.out = torch.nn.Linear(n_hidden, n_output)
            def forward(self, x):
                x=self.hidden(x)
                out =  self.out(x)
                return out
            
        #新建net
        net = Net(n_feature=5, n_hidden=64, n_output=3)

        #载入参数
        net.load_state_dict(torch.load('xiao.pkl'))

        #显示网络
        print("net_restore")
        print(net)

        #--------调试代码---------
        #-----------------cuda加速----------------
        #cuda加速
        net.cuda() 

        loss_func = torch.nn.MSELoss()
        loss_func.to(device)

        #-----------------测试误差-----------------

        # 定义数据库 （输入输出分别是之前的输入输出）-----一次性输入所有测试数据，计算loss总和 
        dataset_test = Data.TensorDataset(self.data_test_torch, self.label_test_torch) 

        # 定义数据加载器
        #loader_test = Data.DataLoader(dataset = dataset_test, batch_size = self.BATCH_SIZE, shuffle = True, num_workers = 2)
        loader_test = Data.DataLoader(dataset = dataset_test, batch_size = self.test_data_number, shuffle = True, num_workers = 2) #一次性把所有数据载入进行批训练

        #将测试集loss画出来(一次性载入所有数据）
        for step,(batch_x, batch_y) in enumerate(loader_test):
            #产生prediction
            prediction = net( batch_x.to(device) )

            #计算总误差
            error_test = prediction - batch_y.to(device) #计算绝对值误差 4157*3

            #计算每项平均误差
            error_test_average = torch.sum(error_test,0)/self.test_data_number
            #error_test_average = torch.sum(error_test,1)

            #取总误差绝对值
            error_test_abs = torch.abs(error_test)

            #总误差绝对值行列相加 
            error_sum = torch.sum(error_test_abs)

            torch.set_printoptions(threshold=1000000)

            #显示每一项误差
            print("error_test")
            print(error_test)

            #显示平均误差
            print("error_test_average")
            print(error_test_average)

            #显示绝对误差总和
            print("error_sum")
            print(error_sum )

            #计算总loss
            loss = loss_func(prediction, batch_y.to(device))

            print("测试集loss")
            print(loss)
        
        return net

if __name__ == '__main__':

    #-----------------初次训练网络---------------

    #载入数据
    train_data = np.load("data_final_2019_4_1.npy")

    #网络训练 
    dnn = neural_network(train_data,50)
    dnn.dnn()

    #将误差函数画出来
    dnn.plot_error() 
    dnn.plot_loss()

    ##-----------------重新载入网络---------------

    ##载入数据
    #train_data = np.load("data_final.npy")

    ##重载网络&显示训练结果
    #dnn = neural_network(train_data,20)
    #net = dnn.restore_params()


    

    








