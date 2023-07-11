from GuiPanel import Ui_MainWindow
from PyQt5 import QtWidgets
import numpy as np
from queue import Queue

class Plot_Graphs(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self):
        super(Plot_Graphs, self).__init__()
        self.queue_IMU = Queue(maxsize=0)
        self.setupUi(self)
        global PlotCounter
        self.historyLength = 1600  # 横坐标长度
        self.historyLength_EEG = self.historyLength * 5  # 横坐标长度
        # data=np.zeros(historyLength).__array__('d')#把数组长度定下来
        # setup empty array lengths
        self.data_A = np.zeros((self.historyLength, 9))  # 把数组长度定下来
        self.data_B = np.zeros((self.historyLength, 9))  # 把数组长度定下来

        self.PlotInit_Counter = 0
        self.PlotFrame_Counter = 0
        self.PlotInit_Counter_Attitude = 0
        self.PlotFrame_Counter_Attitude = 0

    def Plot_Init(self):
        self.GraphAcc.showGrid(x=True, y=True)  # 把X和Y的表格打开
        self.GraphAcc.setRange(xRange=[0, self.historyLength], padding=0)
        self.GraphAcc.setLabel(axis='left', text='Acc (g)')  # 靠左
        self.GraphAcc.setLabel(axis='bottom', text='Samples (#)')
        self.GraphAcc.setTitle('Accelerometer - Ears - Left (RGB) Right (MYC)')  # 表格的名字
        self.curve1 = self.GraphAcc.plot(pen="r", name="y1")  # 绘制一个图形
        self.curve2 = self.GraphAcc.plot(pen="g", name="y2")  # 绘制一个图形
        self.curve3 = self.GraphAcc.plot(pen="b", name="y3")  # 绘制一个图形
        self.curve101 = self.GraphAcc.plot(pen="m", name="y1")  # 绘制一个图形
        self.curve102 = self.GraphAcc.plot(pen="y", name="y2")  # 绘制一个图形
        self.curve103 = self.GraphAcc.plot(pen="c", name="y3")  # 绘制一个图形

        self.GraphGyro.showGrid(x=True, y=True)  # 把X和Y的表格打开
        self.GraphGyro.setRange(xRange=[0, self.historyLength], padding=0)
        self.GraphGyro.setLabel(axis='left', text='Gyro (d/s)')  # 靠左
        self.GraphGyro.setLabel(axis='bottom', text='Samples (#)')
        self.GraphGyro.setTitle('Gyroscope - Ears - Left (RGB) Right (MYC)')  # 表格的名字
        self.curve4 = self.GraphGyro.plot(pen="r", name="y1")  # 绘制一个图形
        self.curve5 = self.GraphGyro.plot(pen="g", name="y2")  # 绘制一个图形
        self.curve6 = self.GraphGyro.plot(pen="b", name="y3")  # 绘制一个图形
        self.curve104 = self.GraphGyro.plot(pen="m", name="y1")  # 绘制一个图形
        self.curve105 = self.GraphGyro.plot(pen="y", name="y2")  # 绘制一个图形
        self.curve106 = self.GraphGyro.plot(pen="c", name="y3")  # 绘制一个图形

        self.GraphMag.showGrid(x=True, y=True)  # 把X和Y的表格打开
        self.GraphMag.setRange(xRange=[0, self.historyLength], padding=0)
        self.GraphMag.setLabel(axis='left', text='Mag (uT)')  # 靠左
        self.GraphMag.setLabel(axis='bottom', text='Samples (#)')
        self.GraphMag.setTitle('Magnetometer - Ears - Left (RGB) Right (MYC)')  # 表格的名字
        self.curve7 = self.GraphMag.plot(pen="r", name="y1")  # 绘制一个图形
        self.curve8 = self.GraphMag.plot(pen="g", name="y2")  # 绘制一个图形
        self.curve9 = self.GraphMag.plot(pen="b", name="y3")  # 绘制一个图形
        self.curve107 = self.GraphMag.plot(pen="m", name="y1")  # 绘制一个图形
        self.curve108 = self.GraphMag.plot(pen="y", name="y2")  # 绘制一个图形
        self.curve109 = self.GraphMag.plot(pen="c", name="y3")  # 绘制一个图形

    def Plot_IMUdata(self, IMU_Data):
        # print("about to plot data...")
        if self.PlotInit_Counter < self.historyLength:
            # print(len(IMU_Data))
            self.data_A[self.PlotInit_Counter] = IMU_Data[2:11]
            self.data_B[self.PlotInit_Counter] = IMU_Data[11:20]
            self.PlotInit_Counter = self.PlotInit_Counter + 1
        else:
            self.data_A[:-1] = self.data_A[1:]
            self.data_A[-1:] = IMU_Data[2:11]

            self.data_B[:-1] = self.data_B[1:]
            self.data_B[-1:] = IMU_Data[11:20]

        self.PlotFrame_Counter = self.PlotFrame_Counter + 1
        # self.data[:-1] = self.data[1:]
        # self.data[self.historyLength - 1] = IMU_Data[1:10]
        if self.PlotFrame_Counter >= 50:
            self.PlotFrame_Counter = 0

            # AutoAdjust Yaxis adaptively
            self.AccData_A = self.data_A[:, 0:3]
            self.AccData_B = self.data_B[:, 0:3]

            self.AccY_Max1= max(self.AccData_A.reshape(-1, 1))[0]
            self.AccY_Min1 = min(self.AccData_A.reshape(-1, 1))[0]
            self.AccY_Max2 = max(self.AccData_B.reshape(-1, 1))[0]
            self.AccY_Min2 = min(self.AccData_B.reshape(-1, 1))[0]
            self.AccY_Max = max(self.AccY_Max1, self.AccY_Max2)
            self.AccY_Min = min(self.AccY_Min1, self.AccY_Min2)
            self.GraphAcc.setRange(yRange=[self.AccY_Min, self.AccY_Max])

            self.GyroData_A = self.data_A[:, 3:6]
            self.GyroData_B = self.data_B[:, 3:6]

            self.GyroY_Max1 = max(self.GyroData_A.reshape(-1, 1))[0]
            self.GyroY_Min1 = min(self.GyroData_A.reshape(-1, 1))[0]
            self.GyroY_Max2 = max(self.GyroData_B.reshape(-1, 1))[0]
            self.GyroY_Min2 = min(self.GyroData_B.reshape(-1, 1))[0]
            self.GyroY_Max = max(self.GyroY_Max1, self.GyroY_Max2)
            self.GyroY_Min = min(self.GyroY_Min1, self.GyroY_Min2)
            self.GraphGyro.setRange(yRange=[self.GyroY_Min, self.GyroY_Max])

            self.MagData_A = self.data_A[:, 6:9]
            self.MagData_B = self.data_B[:, 6:9]
            self.MagY_Max1 = max(self.MagData_A.reshape(-1, 1))[0]
            self.MagY_Min1 = min(self.MagData_A.reshape(-1, 1))[0]
            self.MagY_Max2 = max(self.MagData_B.reshape(-1, 1))[0]
            self.MagY_Min2 = min(self.MagData_B.reshape(-1, 1))[0]
            self.MagY_Max = max(self.MagY_Max1, self.MagY_Max2)
            self.MagY_Min = min(self.MagY_Min1, self.MagY_Min2)
            self.GraphMag.setRange(yRange=[self.MagY_Min, self.MagY_Max])

            self.curve1.setData(self.AccData_A[:, 0])
            self.curve2.setData(self.AccData_A[:, 1])
            self.curve3.setData(self.AccData_A[:, 2])
            self.curve101.setData(self.AccData_B[:, 0])
            self.curve102.setData(self.AccData_B[:, 1])
            self.curve103.setData(self.AccData_B[:, 2])

            self.curve4.setData(self.GyroData_A[:, 0])
            self.curve5.setData(self.GyroData_A[:, 1])
            self.curve6.setData(self.GyroData_A[:, 2])
            self.curve104.setData(self.GyroData_B[:, 0])
            self.curve105.setData(self.GyroData_B[:, 1])
            self.curve106.setData(self.GyroData_B[:, 2])

            self.curve7.setData(self.MagData_A[:, 0])
            self.curve8.setData(self.MagData_A[:, 1])
            self.curve9.setData(self.MagData_A[:, 2])
            self.curve107.setData(self.MagData_B[:, 0])
            self.curve108.setData(self.MagData_B[:, 1])
            self.curve109.setData(self.MagData_B[:, 2])