import numpy as np
import torch
import itertools
import datetime

# -------------定义一个模型预测的类型-------------


class MPC:
    # -------------1.初始化-------------
    def __init__(self, x_init, x_desire):
        # ------------初始化赋值------------
        self.x_init = x_init  # 初始状态
        self.x_init = np.reshape(self.x_init, 3).astype(np.float)  # 转化成numpy array

        self.x_desire = x_desire  # 最终状态
        self.x_desire = np.reshape(self.x_desire, 3).astype(np.float)  # 转化成numpy array

        # print("初始化ｘ_init")
        # print(self.x_init)

        # print("初始化x_desire")
        # print(self.x_desire)

        #self.step = 5  # 预测未来步数
        self.step = 2  # 预测未来步数
        self.control_sequence = np.zeros((self.step, 2))  # 实时控制序列
        self.control_final = np.zeros((self.step, 2))  # 最终控制序列

        # ------------初始化参数------------
        self.Q = 100 # 位移调节参数
        self.P = 50  # yaw角调节参数


        # ------------初始化参数------------
        print("导入成功")

    # ------------优化函数------------
    def optimize(self):
        # 计算所有可能的最小值
        # 生成控制量
        u_1 = (1, 2, 3)
        u_2 = (1, 2, 3)
        # u_3 = (1, 2, 3)
        # u_4 = (1, 2, 3)
        # u_5 = (1, 2, 3)

        # 控制量组合所有可能
        #u_all = itertools.product(u_1, u_2, u_3, u_4, u_5)
        u_all = itertools.product(u_1, u_2)

        # 初始化
        best_element = [0, 0, 0]  # 初始值给的很大
        shortest_distance = 10000000  # 初始值给的很大
        final_position = [0, 0, 0]  # 初始值给成零
        final_distance = 0  # 初始值给成零

        for element in u_all:
            
            final_position = self.final_position(self.x_init, element)  # 将两栖机器人五步之后的最终距离得出
            
            #final_distance = self.objective(final_position, self.x_desire)  # 将两栖机器人最终位置和理想位置的距离
            final_distance = self.objective(final_position, self.x_init,self.x_desire)  # 将两栖机器人最终位置和理想位置的距离

            # 得到最小值对应的控制元素
            if final_distance < shortest_distance:
                
                # best_result = result #最佳结果
                shortest_distance = final_distance  # 最佳结果(距离理想位置最近的一条路线)
                best_element = list(element)  # 最佳结果对应的元素

        # 生成最优控制量
        print("shortest_distance")
        print(shortest_distance)

        print("best element")
        print(best_element)

        self.control_final = self.elementToControl(best_element)  # 返回的就是最优控制量

        return self.control_final

    def elementToControl(self, element):
        # 初始化控制序列
        control_final = np.zeros((self.step, 2))  # 最终控制序列

        # 将element翻译成control sequence
        for num in range(self.step):
            if element[num] == 1:
                control_final[num, :] = [1, 0]  # 直行
            elif element[num] == 2:
                control_final[num, :] = [0, 1]  # 右转
            elif element[num] == 3:
                control_final[num, :] = [0, -1]  # 左转

        return control_final

    # 获得五步之后的最终位置（verified)
    def final_position(self, x_init, element):  # x_init=初始位置，element
        # 初始化
        temp_position = 0
        final_position = 0
        delta_position = 0

        self.control_sequence = np.zeros((self.step, 2))  # 实时控制序列

        # 将element翻译成control sequence
        for num in range(self.step):
            if element[num] == 1:
                self.control_sequence[num, :] = [1, 0]  # 直行
            elif element[num] == 2:
                self.control_sequence[num, :] = [0, 1]  # 右转
            elif element[num] == 3:
                self.control_sequence[num, :] = [0, -1]  # 左转

        # #没问题
        # print("control_sequence!!!!!!!!!!!!!!!!!!!!!!!")
        # print(self.control_sequence)

        # 使用神经网络，得到最终位置
        for num in range(self.step):

            # 第一步
            if num == 0:
                #计算一步的位移
                delta_position = self.f(x_init, self.control_sequence[num, :])  # 单步位移
                temp_position = delta_position+x_init #最终状态

                # print("temp_position")
                # print(temp_position)

                continue
            # 第二步及以后
            delta_position = self.f(temp_position, self.control_sequence[num, :]) #单步位移

            temp_position = temp_position+delta_position #最终状态

            # #没问题
            # print("final_position！！！！！！！！！！！！！！！！！！")
            # print(final_position)

            # print("temp_position！！！！！！！！！！！！！！！！！！")
            # print(temp_position)

        #将最终位移数值返回
        final_position = temp_position

        return final_position

    # ------------定义目标代价函数------------
    def objective(self, x_now, x_init, x_desire): #x_now:最终位置 x_init:初始位置 x_desire:最终位置 
        x_now = np.reshape(x_now, 3).astype(np.float)  # np.[1,3]转化成np.[3]

        #计算最终位置和理想位置之间的差
        error_position = np.dot((x_now[0:2]-x_desire[0:2]),(x_now[0:2]-x_desire[0:2]).T)

        # yaw角误差
        error_yaw = self.yawToTarget(x_now,x_init,x_desire)



        # 定义代价函数(只考虑yaw角)
        J = self.P*np.power(error_yaw,2) +self.Q*error_position # 二次型的样子

        return J

    # ------------计算实际角度和理想角度之间的距离------------
    def yawToTarget(self,x_now,x_init,x_desire):
        #初始化
        point_to_target = np.zeros(2) # 理想点和实际点的向量

        #计算实际位置和理想点之间的向量 :目标点-当前点
        point_to_target[0] = x_desire[0]-x_init[0] #方向向量x
        point_to_target[1] = x_desire[1]-x_init[1] #方向向量y

        #计算理想yaw角
        yaw_point_to_target = self.angle(point_to_target[0],point_to_target[1])

        # print("理想角度！！！！！！！！！！！！！！！！！！！！！！！")
        # print(yaw_point_to_target)
        
        #计算实际yaw角
        yaw_now = x_now[2] 

        # print("最终角度！！！！！！！！！！！！！！！！！！！！！！！")
        # print(yaw_now)

        #计算实际yaw角和理想yaw角之间的距离
        error_yaw = yaw_point_to_target - yaw_now 

        return error_yaw

    #求实时和期望的夹角
    def angle(self,point_x,point_y):
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
        #x.dot(y) =  y=∑(ai*bi)

    # ------------定义运动学函数------------
    def f(self, x_t, u_t):  # 0 1 2 3 4 5 对应的效果怎么样，分别用一个if

        # 生成输入量
        data_input = np.append(x_t, u_t)

        # reshape
        data_input_np = np.reshape(data_input, (1, 5)).astype(np.float)

        # torch
        data_input_torch = torch.from_numpy(data_input_np).float()

        # print("data_input_torch")
        # print(data_input_torch)

        # 进行预测
        prediction = self.net(data_input_torch)

        # print("prediction")
        # print(prediction)

        # 将tensor转化回numpy
        return prediction.detach().numpy()

    # ------------重新载入网络------------
    def restore_params(self):
        # 定义网络类型
        class Net(torch.nn.Module):
            def __init__(self, n_feature, n_hidden, n_output):
                super(Net, self).__init__()
                self.hidden = torch.nn.Sequential(
                    torch.nn.Linear(n_feature, n_hidden),
                    torch.nn.LeakyReLU(),
                    torch.nn.Linear(n_hidden, n_hidden),
                    torch.nn.LeakyReLU(),
                    torch.nn.Linear(n_hidden, n_hidden),
                    torch.nn.LeakyReLU()
                )
                self.out = torch.nn.Linear(n_hidden, n_output)

            def forward(self, x):
                x = self.hidden(x)
                out = self.out(x)
                return out

        # 新建net
        self.net = Net(n_feature=5, n_hidden=128, n_output=3)

        # 载入参数
        self.net.load_state_dict(torch.load('ParamsMseAdamNewData.pkl'))

        # 显示网络
        print("net_restore")
        print(self.net)

    # ------------传入理想位置和实际位置------------
    def input_parameters(self, x_init, x_desire):
        # ------------初始化赋值------------
        self.x_init = x_init  # 初始状态
        self.x_init = np.reshape(self.x_init, 3).astype(np.float)  # 转化成numpy array

        self.x_desire = x_desire  # 最终状态
        self.x_desire = np.reshape(self.x_desire, 3).astype(np.float)  # 转化成numpy array


        # print("参数成功传入")
        # print("optimize-x_init")
        # print(self.x_init)

        # print("optimize-x_desire")
        # print(self.x_desire)



